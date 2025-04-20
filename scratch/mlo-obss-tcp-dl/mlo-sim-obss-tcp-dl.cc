/*
 * Copyright (c) 2024
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation;
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 *
 */
#include "ns3/string.h"
#include "ns3/attribute-container.h"
#include "ns3/command-line.h"
#include "ns3/config.h"
#include "ns3/constant-rate-wifi-manager.h"
#include "ns3/eht-configuration.h"
#include "ns3/eht-phy.h"
#include "ns3/frame-exchange-manager.h"
#include "ns3/internet-stack-helper.h"
#include "ns3/log.h"
#include "ns3/mobility-helper.h"
#include "ns3/multi-model-spectrum-channel.h"
#include "ns3/on-off-helper.h"
#include "ns3/packet-sink.h"
#include "ns3/packet-sink-helper.h"
#include "ns3/packet-socket-client.h"
#include "ns3/packet-socket-helper.h"
#include "ns3/packet-socket-server.h"
#include "ns3/udp-client-server-helper.h"
#include "ns3/udp-server.h"
#include "ns3/qos-utils.h"
#include "ns3/rng-seed-manager.h"
#include "ns3/spectrum-wifi-helper.h"
#include "ns3/uinteger.h"
#include "ns3/ap-wifi-mac.h"
#include "ns3/sta-wifi-mac.h"
#include "ns3/wifi-mac-queue.h"
#include "ns3/wifi-net-device.h"
#include "ns3/wifi-phy-common.h"
#include "ns3/wifi-phy.h"
#include "ns3/wifi-utils.h"
#include "ns3/yans-wifi-helper.h"
#include "ns3/ipv4-address-helper.h"
#include "ns3/wifi-tx-stats-helper.h"
#include "ns3/flow-monitor-module.h"
#include "ns3/wifi-phy-rx-trace-helper.h"
#include "ns3/tcp-socket.h"
#define PI 3.1415926535

using namespace ns3;

NS_LOG_COMPONENT_DEFINE("mlo-obss-dl-tcp");
// 自定义回调函数：STA 发送数据包时调用
void NotifyAppTx(Ptr<const Packet> packet)
{
    Time now = Simulator::Now();
    std::cout << "STA Tx at " << now.GetSeconds() << "s, UID " 
              << packet->GetUid() << std::endl;
}

// 自定义回调函数：AP 接收数据包时调用
void NotifyAppRx(Ptr<const Packet> packet)
{
    Time now = Simulator::Now();
    std::cout << "AP Rx at " << now.GetSeconds() << "s, UID " 
              << packet->GetUid() << std::endl;
}

void
GetRxBytes(bool udp, const ApplicationContainer& serverApp, uint32_t payloadSize, std::vector<uint64_t>& rxBytes)
{
    if (udp)
    {
        for (uint32_t i = 0; i < serverApp.GetN(); i++)
        {
            rxBytes[i] = payloadSize * DynamicCast<UdpServer>(serverApp.Get(i))->GetReceived();
        }
    }
    else
    {
        for (uint32_t i = 0; i < serverApp.GetN(); i++)
        {
            rxBytes[i] = DynamicCast<PacketSink>(serverApp.Get(i))->GetTotalRx();
        }
    }
}

void
PrintIntermediateTput(std::vector<uint64_t>& rxBytes_begin,
                      std::vector<uint64_t>& rxBytes_end,
                      bool udp,
                      const ApplicationContainer& serverApp,
                      uint32_t payloadSize,
                      Time delta,
                      Time tputInterval,
                      Time simulationTime)
{
    Simulator::Schedule(delta, &GetRxBytes, udp, serverApp, payloadSize, std::ref(rxBytes_begin));
    Time now = Simulator::Now();
    std::cout << "[" << (now - tputInterval).As(Time::S) << " - " << now.As(Time::S)
              << "] Throughput (Mbit/s):";
    GetRxBytes(udp, serverApp, payloadSize, rxBytes_end);
    for (std::size_t i = 0; i < rxBytes_end.size(); i++)
    {
        std::cout << "\t\t(" << i << ") "
                  << (rxBytes_end[i] - rxBytes_begin[i]) * 8. / tputInterval.GetMicroSeconds(); // Mbit/s
    }
    std::cout << std::endl;

    rxBytes_end.swap(rxBytes_end);

    if (now < (simulationTime - NanoSeconds(1)))
    {
        Simulator::Schedule(Min(tputInterval + delta, simulationTime - now - NanoSeconds(1)),
                            &PrintIntermediateTput,
                            std::ref(rxBytes_begin),
                            std::ref(rxBytes_end),
                            udp,
                            serverApp,
                            payloadSize,
                            delta,
                            tputInterval,
                            simulationTime);
    }
}


int
main(int argc, char* argv[])
{
    uint32_t seedNumber = 1;
    uint8_t nLinks = 2;
    RngSeedManager::SetSeed(seedNumber);
    RngSeedManager::SetRun(seedNumber);
    double txPower = 1; 
    bool useRts{false};
    // std::string rateCtrl{"constant"};
    std::string rateCtrl{"constant"};
    int gi = 800;
    uint16_t mpduBufferSize{256};
    Time simulationTime{"10s"};
    std::string dlAckSeqType{"NO-OFDMA"};
    size_t nStaMlds{1};
    std::vector<size_t> nStaSlds{1, 1};
    uint32_t payloadSize = 1500; // must fit in the max TX duration when transmitting at MCS 0 over an RU of 26 tones
    if (useRts)
    {
        Config::SetDefault("ns3::WifiRemoteStationManager::RtsCtsThreshold", StringValue("0"));
        Config::SetDefault("ns3::WifiDefaultProtectionManager::EnableMuRts", BooleanValue(true));
    }

    NodeContainer apNodes;
    NodeContainer mldNodes;
    NodeContainer sldNodes2;
    NodeContainer sldNodes5;
    apNodes.Create(3);
    mldNodes.Create(nStaMlds);
    sldNodes2.Create(nStaSlds[0]);
    sldNodes5.Create(nStaSlds[1]);
    NetDeviceContainer apDev;
    NetDeviceContainer mldDev;
    NetDeviceContainer sldDev2;
    NetDeviceContainer sldDev5;

    /* WIFI Configuration */
    WifiHelper wifi;
    WifiHelper wifi2;
    WifiHelper wifi5;
    wifi.SetStandard(WIFI_STANDARD_80211be);
    wifi2.SetStandard(WIFI_STANDARD_80211ax);
    wifi5.SetStandard(WIFI_STANDARD_80211ax);
    std::vector<int> mcs{2, 11};
    std::vector<int> bandwidth{20, 40};
    std::string dataModeStr;
    uint64_t nonHtRefRateMbps;
    std::string ctrlRateStr; 
    if (rateCtrl == "constant") {
        for (uint8_t i = 0; i < nLinks; ++i) {
            if (i == 0) {
            dataModeStr = "EhtMcs" + std::to_string(mcs[i]);
            nonHtRefRateMbps = EhtPhy::GetNonHtReferenceRate(mcs[i]) / 1e6;
            ctrlRateStr = "ErpOfdmRate" + std::to_string(nonHtRefRateMbps) + "Mbps";
            std::cout << "Link " << std::to_string(i) <<" ControlRate: " << ctrlRateStr << " DataMode: " << dataModeStr << std::endl;
            wifi.SetRemoteStationManager(i, 
                                            "ns3::ConstantRateWifiManager",
                                            "DataMode", StringValue(dataModeStr),
                                            "ControlMode", StringValue(ctrlRateStr));
            dataModeStr = "HeMcs" + std::to_string(mcs[i]);
            nonHtRefRateMbps = HePhy::GetNonHtReferenceRate(mcs[i]) / 1e6;
            ctrlRateStr = "ErpOfdmRate" + std::to_string(nonHtRefRateMbps) + "Mbps";
            std::cout << "2.4G OBSS: ControlRate: " << ctrlRateStr << " DataMode: " << dataModeStr << std::endl;
            wifi2.SetRemoteStationManager("ns3::ConstantRateWifiManager",
                                                "DataMode", StringValue(dataModeStr),
                                                "ControlMode", StringValue(ctrlRateStr));
            }
            else {
            dataModeStr = "EhtMcs" + std::to_string(mcs[i]);
            nonHtRefRateMbps = EhtPhy::GetNonHtReferenceRate(mcs[i]) / 1e6;
            ctrlRateStr = "OfdmRate" + std::to_string(nonHtRefRateMbps) + "Mbps";
            std::cout << "Link " << std::to_string(i) <<" ControlRate: " << ctrlRateStr << " DataMode: " << dataModeStr << std::endl;
            wifi.SetRemoteStationManager(i, 
                                            "ns3::ConstantRateWifiManager",
                                            "DataMode", StringValue(dataModeStr),
                                            "ControlMode", StringValue(ctrlRateStr));
            dataModeStr = "HeMcs" + std::to_string(mcs[i] > 11 ? 11 : mcs[i]);
            nonHtRefRateMbps = HePhy::GetNonHtReferenceRate(mcs[i] > 11 ? 11 : mcs[i]) / 1e6;
            ctrlRateStr = "OfdmRate" + std::to_string(nonHtRefRateMbps) + "Mbps";
            std::cout << "5G OBSS: ControlRate: " << ctrlRateStr << " DataMode: " << dataModeStr << std::endl;
            wifi5.SetRemoteStationManager("ns3::ConstantRateWifiManager",
                                                "DataMode", StringValue(dataModeStr),
                                                "ControlMode", StringValue(ctrlRateStr));
            }
        }
    } else if (rateCtrl == "minstrel") {
        for (uint8_t i = 0; i < nLinks; ++i) {
            wifi.SetRemoteStationManager(i, 
                                            "ns3::MinstrelHtWifiManager");
            if (i == 0) {
            wifi2.SetRemoteStationManager("ns3::MinstrelHtWifiManager");
            }
            else {
            wifi5.SetRemoteStationManager("ns3::MinstrelHtWifiManager");
            }
        }
    }

    // PD SR
    // wifi.SetObssPdAlgorithm("ns3::ConstantObssPdAlgorithm", "ObssPdLevel", DoubleValue(-72.0));

    /* PropagationLossModel Configuration */
    Ptr<MultiModelSpectrumChannel> spectrumChannel_2 = CreateObject<MultiModelSpectrumChannel>();
    Ptr<LogDistancePropagationLossModel> lossModel_2 = CreateObject<LogDistancePropagationLossModel>();
    lossModel_2->SetAttribute("Exponent", DoubleValue(2.0));
    lossModel_2->SetAttribute("ReferenceDistance", DoubleValue(1.0));
    lossModel_2->SetAttribute("ReferenceLoss", DoubleValue(40.046));
    spectrumChannel_2->AddPropagationLossModel(lossModel_2);
    
    Ptr<MultiModelSpectrumChannel> spectrumChannel_5 = CreateObject<MultiModelSpectrumChannel>();
    Ptr<LogDistancePropagationLossModel> lossModel_5 = CreateObject<LogDistancePropagationLossModel>();
    lossModel_5->SetAttribute("Exponent", DoubleValue(3.5));
    lossModel_5->SetAttribute("ReferenceDistance", DoubleValue(1.0));
    lossModel_5->SetAttribute("ReferenceLoss", DoubleValue(50));
    spectrumChannel_5->AddPropagationLossModel(lossModel_5);

    /* PHY Configuration */
    SpectrumWifiPhyHelper phy(nLinks);
    phy.SetPcapDataLinkType(WifiPhyHelper::DLT_IEEE802_11_RADIO);
    phy.SetErrorRateModel("ns3::TableBasedErrorRateModel");
    phy.AddChannel(spectrumChannel_2, WIFI_SPECTRUM_2_4_GHZ);
    phy.AddChannel(spectrumChannel_5, WIFI_SPECTRUM_5_GHZ);
    for (uint8_t i = 0; i < nLinks; ++i) {
        if (i == 0) phy.AddPhyToFreqRangeMapping(i, WIFI_SPECTRUM_2_4_GHZ);
        else phy.AddPhyToFreqRangeMapping(i, WIFI_SPECTRUM_5_GHZ);
    }
    phy.Set("TxPowerStart", DoubleValue(txPower));
    phy.Set("TxPowerEnd", DoubleValue(txPower));
    for (uint8_t i = 0; i < nLinks; ++i)
    {
        std::string channelSetting = "{0, " + std::to_string(bandwidth[i]);
        if (i == 0) phy.Set(i, "ChannelSettings", StringValue(channelSetting + ", BAND_2_4GHZ, 0}"));
        else phy.Set(i, "ChannelSettings", StringValue(channelSetting + ", BAND_5GHZ, 0}"));
    }
    
    // SLD PHY
    SpectrumWifiPhyHelper phySld2(1);
    phySld2.SetPcapDataLinkType(WifiPhyHelper::DLT_IEEE802_11_RADIO);
    phySld2.SetErrorRateModel("ns3::TableBasedErrorRateModel");
    phySld2.Set("TxPowerStart", DoubleValue(txPower));
    phySld2.Set("TxPowerEnd", DoubleValue(txPower));
    phySld2.AddChannel(spectrumChannel_2, WIFI_SPECTRUM_2_4_GHZ);
    phySld2.Set("ChannelSettings", StringValue("{0, " + std::to_string(bandwidth[0]) + ", BAND_2_4GHZ, 0}"));

    SpectrumWifiPhyHelper phySld5(1);
    phySld5.SetPcapDataLinkType(WifiPhyHelper::DLT_IEEE802_11_RADIO);
    phySld5.SetErrorRateModel("ns3::TableBasedErrorRateModel");
    phySld5.Set("TxPowerStart", DoubleValue(txPower));
    phySld5.Set("TxPowerEnd", DoubleValue(txPower));
    phySld5.AddChannel(spectrumChannel_5, WIFI_SPECTRUM_5_GHZ);
    phySld5.Set("ChannelSettings", StringValue("{0, " + std::to_string(bandwidth[1]) + ", BAND_5GHZ, 0}"));

    /* MAC Configuration */
    WifiMacHelper mac;
    Ssid bssSsid = Ssid("AP-MLD");
    Ssid obssSsid2 = Ssid("AP-2G");
    Ssid obssSsid5 = Ssid("AP-5G");
    // 1. STA MAC 设置
    mac.SetType("ns3::StaWifiMac",
                "Ssid", SsidValue(bssSsid),
                "ActiveProbing", BooleanValue(false));
    mldDev = wifi.Install(phy, mac, mldNodes);

    mac.SetType("ns3::StaWifiMac",
                "Ssid", SsidValue(obssSsid2),
                "ActiveProbing", BooleanValue(false));
    sldDev2 = wifi2.Install(phySld2, mac, sldNodes2);

    mac.SetType("ns3::StaWifiMac",
                "Ssid", SsidValue(obssSsid5),
                "ActiveProbing", BooleanValue(false));
    sldDev5 = wifi5.Install(phySld5, mac, sldNodes5);

    // print mac address of mld 
    Ptr<WifiMac> mac1 = DynamicCast<WifiNetDevice>(mldDev.Get(0))->GetMac();
    std::cout << "MLD MAC: " << mldDev.Get(0)->GetAddress() << std::endl;
    for (uint8_t i = 0; i < nLinks; ++i) {
        auto fem = mac1->GetFrameExchangeManager(i);
        std::cout << "\t mldDevice " << "linkId " << std::to_string(i) << " mac address: " << fem->GetAddress() << std::endl;
    }

    // print mac address of slds
    Ptr<WifiMac> mac_sld;
    for (auto i = 0; i < nLinks; ++i) { 
        if (nStaSlds[i])
            std::cout << "SLD on Link " << std::to_string(i) << ":" << std::endl;
        for (auto j = 0; j < nStaSlds[i]; ++j) {
            if (i == 0) mac_sld = DynamicCast<WifiNetDevice>(sldDev2.Get(j))->GetMac();
            else mac_sld = DynamicCast<WifiNetDevice>(sldDev5.Get(j))->GetMac();
            auto fem = mac_sld->GetFrameExchangeManager();
            std::cout << "\t SLD-" << j << ": " << fem->GetAddress() << std::endl;
        }
    }
    // 2. AP MAC 设置
    uint64_t beaconInterval = 1024 * 100;
    mac.SetType("ns3::ApWifiMac",
                "BeaconInterval", TimeValue(MicroSeconds(beaconInterval)),
                "EnableBeaconJitter", BooleanValue(false),
                "Ssid", SsidValue(bssSsid));
    apDev = wifi.Install(phy, mac, apNodes.Get(0));
    
    mac.SetType("ns3::ApWifiMac",
                "BeaconInterval", TimeValue(MicroSeconds(beaconInterval)),
                "EnableBeaconJitter", BooleanValue(false),
                "Ssid", SsidValue(obssSsid2));
    apDev.Add(wifi2.Install(phySld2, mac, apNodes.Get(1)));
    
    mac.SetType("ns3::ApWifiMac",
                "BeaconInterval", TimeValue(MicroSeconds(beaconInterval)),
                "EnableBeaconJitter", BooleanValue(false),
                "Ssid", SsidValue(obssSsid5));               
    apDev.Add(wifi5.Install(phySld5, mac, apNodes.Get(2)));

    // print mac address of ap
    std::cout << "AP0 MAC: " << apDev.Get(0)->GetAddress()<< std::endl;
    Ptr<WifiMac> mac2 = DynamicCast<WifiNetDevice>(apDev.Get(0))->GetMac();
    for (uint8_t i = 0; i < nLinks; ++i) {
        auto fem = mac2->GetFrameExchangeManager(i);
        std::cout << "\t apDevice " << "linkId " << std::to_string(i) << " mac address: " << fem->GetAddress() << std::endl;
    }
    std::cout << "AP1 MAC: " << apDev.Get(1)->GetAddress()<< std::endl;
    std::cout << "AP2 MAC: " << apDev.Get(2)->GetAddress()<< std::endl;
    NetDeviceContainer devices;
    devices.Add(apDev);
    devices.Add(mldDev);
    devices.Add(sldDev2);
    devices.Add(sldDev5);
    wifi.AssignStreams(devices, seedNumber);

    // Set guard interval, MPDU buffer size, MaxAmsduSize, MaxAmpduSize
    int16_t AmpduSize = 64;
    Config::Set("/NodeList/*/DeviceList/*/$ns3::WifiNetDevice/HeConfiguration/GuardInterval",
                    TimeValue(NanoSeconds(gi)));
    Config::Set("/NodeList/*/DeviceList/*/$ns3::WifiNetDevice/Mac/BE_MaxAmsduSize",
            UintegerValue(0 * (payloadSize + 150)));
    Config::Set("/NodeList/*/DeviceList/*/$ns3::WifiNetDevice/Mac/BE_MaxAmpduSize",
                    UintegerValue(AmpduSize * (payloadSize + 150)));
    Config::Set("/NodeList/*/DeviceList/*/$ns3::WifiNetDevice/Mac/MpduBufferSize",
                            UintegerValue(mpduBufferSize));

    
    /* Mobility Configuration */
    MobilityHelper mobility;
    Ptr<ListPositionAllocator> positionAlloc = CreateObject<ListPositionAllocator>();
    Ptr<UniformRandomVariable> Rng = CreateObject<UniformRandomVariable> ();
    Rng->SetAttribute ("Min", DoubleValue (0));
    Rng->SetAttribute ("Max", DoubleValue (10));
    mobility.SetMobilityModel("ns3::ConstantPositionMobilityModel");
    positionAlloc->Add(Vector(0.0, 0.0, 0.0));
    positionAlloc->Add(Vector(10.0, 0.0, 0.0));
    positionAlloc->Add(Vector(0.0, 10.0, 0.0));
    positionAlloc->Add(Vector(0.0, 0.0, 1.0));
    positionAlloc->Add(Vector(10.0, 0.0, 1.0));
    positionAlloc->Add(Vector(0.0, 10.0, 1.0));
    // for (uint8_t i = 0; i < nStaMlds + nStaSlds[0] + nStaSlds[1]; ++i) {
    //     if (i == 0) positionAlloc->Add(Vector(distance, 0.0, 0.0));
    //     else positionAlloc->Add(Vector(Rng->GetValue()+200, Rng->GetValue(), 0));
    // }
    mobility.SetPositionAllocator(positionAlloc);
    NodeContainer allNodes(apNodes, mldNodes, sldNodes2, sldNodes5);
    mobility.Install(allNodes);
    
    /* Internet stack*/
    InternetStackHelper stack;
    stack.Install(allNodes);
    seedNumber += stack.AssignStreams(allNodes, seedNumber);

    Ipv4AddressHelper address;
    address.SetBase("10.0.0.0", "255.255.255.0");
    Ipv4InterfaceContainer apNodeInterface = address.Assign(apDev.Get(0));
    Ipv4InterfaceContainer mldNodeInterface = address.Assign(mldDev);
    address.SetBase("10.0.1.0", "255.255.255.0");
    Ipv4InterfaceContainer apNodeInterface2 = address.Assign(apDev.Get(1));
    Ipv4InterfaceContainer sldNodeInterface2 = address.Assign(sldDev2);
    address.SetBase("10.0.2.0", "255.255.255.0");
    Ipv4InterfaceContainer apNodeInterface5 = address.Assign(apDev.Get(2));
    Ipv4InterfaceContainer sldNodeInterface5 = address.Assign(sldDev5);
    // print Ip address of devices
    std::cout << "AP0 IP: " << apNodeInterface.GetAddress(0) << std::endl;
    std::cout << "AP1 IP: " << apNodeInterface2.GetAddress(0) << std::endl;
    std::cout << "AP2 IP: " << apNodeInterface5.GetAddress(0) << std::endl;
    std::cout << "STA IP: " << std::endl;
    for (size_t i = 0; i < nStaMlds; ++i) {
        std::cout << "  MLD-" << std::to_string(i) + ": " << mldNodeInterface.GetAddress(i) << std::endl;
    }
    for (size_t i = 0; i < nStaSlds[0];  ++i) {
        std::cout << "  Link 0 SLD-" << std::to_string(i) + ": " << sldNodeInterface2.GetAddress(i) << std::endl;
    }
    for (size_t i = 0; i < nStaSlds[1];  ++i) {
        std::cout << "  Link 1 SLD-" << std::to_string(i) + ": " << sldNodeInterface5.GetAddress(i) << std::endl;
    }
    /* Setting applications */
    ApplicationContainer dlserverApp;
    ApplicationContainer dlserverAppObss2;
    ApplicationContainer dlserverAppObss5;
    // 1. DL TCP configure
    // DL tcp flow
    uint16_t port = 50000;
    Address localAddress(InetSocketAddress(Ipv4Address::GetAny(), port));
    PacketSinkHelper packetSinkHelper("ns3::TcpSocketFactory", localAddress);
    dlserverApp = packetSinkHelper.Install(mldNodes);
    seedNumber += packetSinkHelper.AssignStreams(mldNodes, seedNumber);
    dlserverApp.Start(Seconds(0.0));
    dlserverApp.Stop(simulationTime + Seconds(1.0));

    dlserverAppObss2 = packetSinkHelper.Install(sldNodes2);
    seedNumber += packetSinkHelper.AssignStreams(sldNodes2, seedNumber);
    dlserverAppObss2.Start(Seconds(0.2));
    dlserverAppObss2.Stop(simulationTime + Seconds(1.0));

    dlserverAppObss5 = packetSinkHelper.Install(sldNodes5);
    seedNumber += packetSinkHelper.AssignStreams(sldNodes5, seedNumber);
    dlserverAppObss5.Start(Seconds(0.4));
    dlserverAppObss5.Stop(simulationTime + Seconds(1.0));

    // AP 0
    const auto maxLoad2 = EhtPhy::GetDataRate(mcs[0], bandwidth[0] , NanoSeconds(gi), 1);
    const auto maxLoad5 = EhtPhy::GetDataRate(mcs[1], bandwidth[1] , NanoSeconds(gi), 1);
    const auto maxLoad =  maxLoad2 + maxLoad5; 
    std::cout << "maxload = " << std::to_string(maxLoad/1e6) << "Mbps; 2.4 GHz: " <<  std::to_string(maxLoad2/1e6) << "Mbps, 5 GHz: " << std::to_string(maxLoad5/1e6) << "Mbps" << std::endl;
    OnOffHelper onoff("ns3::TcpSocketFactory", Ipv4Address::GetAny());
    onoff.SetAttribute("OnTime",
                        StringValue("ns3::ConstantRandomVariable[Constant=1]"));
    onoff.SetAttribute("OffTime",
                        StringValue("ns3::ConstantRandomVariable[Constant=0]"));
    onoff.SetAttribute("PacketSize", UintegerValue(payloadSize));
    onoff.SetAttribute("DataRate", DataRateValue(maxLoad));
    AddressValue remoteAddress(InetSocketAddress(mldNodeInterface.GetAddress(0), port));
    onoff.SetAttribute("Remote", remoteAddress);
    ApplicationContainer clientApp = onoff.Install(apNodes.Get(0));
    seedNumber += onoff.AssignStreams(apNodes.Get(0), seedNumber);
    clientApp.Start(Seconds(1.0));
    clientApp.Stop(simulationTime + Seconds(1.0));

    // AP 1
    onoff.SetAttribute("OnTime",
                        StringValue("ns3::ConstantRandomVariable[Constant=1]"));
    onoff.SetAttribute("OffTime",
                        StringValue("ns3::ConstantRandomVariable[Constant=0]"));
    onoff.SetAttribute("PacketSize", UintegerValue(payloadSize));
    onoff.SetAttribute("DataRate", StringValue("1.1Mb/s"));
    // onoff.SetAttribute("DataRate", DataRateValue(maxLoad));
    AddressValue remoteAddress2(InetSocketAddress(sldNodeInterface2.GetAddress(0), port));
    onoff.SetAttribute("Remote", remoteAddress2);
    ApplicationContainer clientApp2 = onoff.Install(apNodes.Get(1));
    seedNumber += onoff.AssignStreams(apNodes.Get(1), seedNumber);
    clientApp2.Start(Seconds(1.2));
    clientApp2.Stop(simulationTime + Seconds(1.0));

    // AP 2
    onoff.SetAttribute("OnTime",
                        StringValue("ns3::ConstantRandomVariable[Constant=1]"));
    onoff.SetAttribute("OffTime",
                        StringValue("ns3::ConstantRandomVariable[Constant=0]"));
    onoff.SetAttribute("PacketSize", UintegerValue(payloadSize));
    onoff.SetAttribute("DataRate", StringValue("1.1Mb/s"));
    // onoff.SetAttribute("DataRate", DataRateValue(maxLoad));
    AddressValue remoteAddress5(InetSocketAddress(sldNodeInterface5.GetAddress(0), port));
    onoff.SetAttribute("Remote", remoteAddress5);
    ApplicationContainer clientApp5 = onoff.Install(apNodes.Get(2));
    seedNumber += onoff.AssignStreams(apNodes.Get(2), seedNumber);
    clientApp5.Start(Seconds(1.2));
    clientApp5.Stop(simulationTime + Seconds(1.0));

    // Enable TID-to-Link Mapping for AP and MLD STAs
    for (auto i = mldDev.Begin(); i != mldDev.End(); ++i)
    {
        auto wifiDev = DynamicCast<WifiNetDevice>(*i);
        wifiDev->GetMac()->GetEhtConfiguration()->SetAttribute("TidToLinkMappingNegSupport",
                                                               EnumValue(WifiTidToLinkMappingNegSupport::ANY_LINK_SET));
    }

    std::string mldMappingStr = "0,1,2,3,4,5,6,7 0,1";
    for (auto i = mldDev.Begin(); i != mldDev.End(); ++i)
    {
        auto wifiDev = DynamicCast<WifiNetDevice>(*i);
        wifiDev->GetMac()->SetAttribute("ActiveProbing", BooleanValue(true));
        wifiDev->GetMac()->GetEhtConfiguration()->SetAttribute("TidToLinkMappingDl", StringValue(mldMappingStr));
        wifiDev->GetMac()->GetEhtConfiguration()->SetAttribute("TidToLinkMappingUl",StringValue(mldMappingStr));
    }

    Config::Set("/NodeList/0/DeviceList/*/$ns3::WifiNetDevice/Mac/BE_Txop/MaxGroupSize", UintegerValue(1));
    Config::Set("/NodeList/0/DeviceList/*/$ns3::WifiNetDevice/Mac/BE_Txop/Mode", UintegerValue(0));
    Config::Set("/NodeList/0/DeviceList/*/$ns3::WifiNetDevice/Mac/BE_Txop/GridSearchEnable", BooleanValue(false));

    phy.EnablePcap("ap0-trace", apDev.Get(0));
    phySld2.EnablePcap("ap1-trace", apDev.Get(1));
    phySld5.EnablePcap("ap2-trace", apDev.Get(2));
    phy.EnablePcap("mld-trace", mldDev.Get(0));
    phySld2.EnablePcap("sld2-trace", sldDev2.Get(0));
    phySld5.EnablePcap("sld5-trace", sldDev5.Get(0));

    Ptr<FlowMonitor> flowMonitor;
    FlowMonitorHelper flowHelper;
    flowMonitor = flowHelper.Install(apNodes.Get(0));


    std::vector<uint64_t> dlCumulRxBytes_begin(dlserverApp.GetN(), 0);
    std::vector<uint64_t> dlCumulRxBytes_end(dlserverApp.GetN(), 0);
    Time delta{Seconds(1.0)};
    Time tputInterval{Seconds(2.0)}; // interval for detailed throughput measurement
    Simulator::Schedule(Seconds(3.0), &PrintIntermediateTput, std::ref(dlCumulRxBytes_begin), std::ref(dlCumulRxBytes_end), false, dlserverApp, payloadSize, delta, tputInterval, simulationTime + Seconds(1.0));
    std::vector<uint64_t> rx_totalbytes(dlserverApp.GetN(), 0);
    Simulator::Schedule(simulationTime + Seconds(1.0), &GetRxBytes, false, dlserverApp, payloadSize, std::ref(rx_totalbytes));

    Simulator::Stop(simulationTime + Seconds(1.0));
    Simulator::Run();
    std::cout << "Total DL Throughput: \t\t\t" << rx_totalbytes[0] * 8 / simulationTime.GetMicroSeconds() << " Mbit/s" << std::endl;

    
    //  // 统计丢包数
    //  flowMonitor->CheckForLostPackets();
    //  Ptr<Ipv4FlowClassifier> classifier = DynamicCast<Ipv4FlowClassifier>(flowHelper.GetClassifier());
    //  std::map<FlowId, FlowMonitor::FlowStats> stats = flowMonitor->GetFlowStats();
 
    //  double totalTxPackets = 0, totalRxPackets = 0, totalLostPackets = 0, totalThroughput = 0;
 
    //  for (auto iter = stats.begin(); iter != stats.end(); ++iter) {
    //     totalTxPackets += iter->second.txPackets;
    //     totalRxPackets += iter->second.rxPackets;
    //     totalLostPackets += iter->second.lostPackets;
    //     double flowThroughput = (iter->second.rxBytes * 8.0) / simulationTime.GetSeconds() / 1e6; // Mbps
    //     totalThroughput += flowThroughput;
    //  }
 
    //  // 计算丢包率
    //  double packetLossRate = (totalLostPackets / totalTxPackets) * 100.0;
    //  std::cout << "Total Tx Packets: " << totalTxPackets << std::endl;
    //  std::cout << "Total Rx Packets: " << totalRxPackets << std::endl;
    //  std::cout << "Total Lost Packets: " << totalLostPackets << std::endl;
    //  std::cout << "Packet Loss Rate: " << packetLossRate << " %" << std::endl;
    //  std::cout << "Throughput: " << totalThroughput << " Mbps" << std::endl;
    
    Simulator::Destroy();
    return 0;
}