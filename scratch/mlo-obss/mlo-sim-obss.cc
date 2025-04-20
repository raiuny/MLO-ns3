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

#define PI 3.1415926535

using namespace ns3;

NS_LOG_COMPONENT_DEFINE("mlo-obss");

void NotifyAppTx(Ptr<const Packet> packet)
{
    Time now = Simulator::Now();
    std::cout << "STA Tx at " << now.GetSeconds() << "s, UID " 
              << packet->GetUid() << std::endl;
}

std::vector<uint64_t>
GetRxBytes(bool udp, const ApplicationContainer& serverApp, uint32_t payloadSize)
{
    std::vector<uint64_t> rxBytes(serverApp.GetN(), 0);
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
    return rxBytes;
}

void
PrintIntermediateTput(std::vector<uint64_t>& rxBytes,
                      bool udp,
                      const ApplicationContainer& serverApp,
                      uint32_t payloadSize,
                      Time tputInterval,
                      Time simulationTime)
{
    auto newRxBytes = GetRxBytes(udp, serverApp, payloadSize);
    Time now = Simulator::Now();
    std::cout << "[" << (now - tputInterval).As(Time::S) << " - " << now.As(Time::S)
              << "] Per-STA Throughput (Mbit/s):";

    for (std::size_t i = 0; i < newRxBytes.size(); i++)
    {
        std::cout << "\t\t(" << i << ") "
                  << (newRxBytes[i] - rxBytes[i]) * 8. / tputInterval.GetMicroSeconds(); // Mbit/s
    }
    std::cout << std::endl;

    rxBytes.swap(newRxBytes);

    if (now < (simulationTime - NanoSeconds(1)))
    {
        Simulator::Schedule(Min(tputInterval, simulationTime - now - NanoSeconds(1)),
                            &PrintIntermediateTput,
                            rxBytes,
                            udp,
                            serverApp,
                            payloadSize,
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
    double txPower = 16; 
    bool udp{true};
    bool downlink{true};
    bool uplink{false};
    bool useRts{false};
    // std::string rateMode{"constant"};
    std::string rateMode{"minstrel"};
    int gi = 800;
    uint16_t mpduBufferSize{256};
    Time simulationTime{"10s"};
    std::string dlAckSeqType{"NO-OFDMA"};
    meter_u distance{1.0};
    size_t nStaMlds{1};
    std::vector<size_t> nStaSlds{1, 1};
    bool enableUlOfdma{false};
    bool enableBsrp{false};
    uint32_t payloadSize = 700; // must fit in the max TX duration when transmitting at MCS 0 over an RU of 26 tones
    Time tputInterval{0}; // interval for detailed throughput measurement
    Time accessReqInterval{0};
    if (useRts)
    {
        Config::SetDefault("ns3::WifiRemoteStationManager::RtsCtsThreshold", StringValue("0"));
        Config::SetDefault("ns3::WifiDefaultProtectionManager::EnableMuRts", BooleanValue(true));
    }

    if (dlAckSeqType == "ACK-SU-FORMAT")
    {
        Config::SetDefault("ns3::WifiDefaultAckManager::DlMuAckSequenceType",
                           EnumValue(WifiAcknowledgment::DL_MU_BAR_BA_SEQUENCE));
    }
    else if (dlAckSeqType == "MU-BAR")
    {
        Config::SetDefault("ns3::WifiDefaultAckManager::DlMuAckSequenceType",
                           EnumValue(WifiAcknowledgment::DL_MU_TF_MU_BAR));
    }
    else if (dlAckSeqType == "AGGR-MU-BAR")
    {
        Config::SetDefault("ns3::WifiDefaultAckManager::DlMuAckSequenceType",
                           EnumValue(WifiAcknowledgment::DL_MU_AGGREGATE_TF));
    }
    else if (dlAckSeqType != "NO-OFDMA")
    {
        NS_ABORT_MSG("Invalid DL ack sequence type (must be NO-OFDMA, ACK-SU-FORMAT, MU-BAR or "
                     "AGGR-MU-BAR)");
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
    std::vector<int> mcs{2, 6};
    std::vector<int> bandwidth{20, 20};
    std::string dataModeStr;
    uint64_t nonHtRefRateMbps;
    std::string ctrlRateStr; 
    if (rateMode == "constant") {
        for (uint8_t i = 0; i < nLinks; ++i) {
            dataModeStr = "EhtMcs" + std::to_string(mcs[i]);
            nonHtRefRateMbps = EhtPhy::GetNonHtReferenceRate(mcs[i]) / 1e6;
            ctrlRateStr = "OfdmRate" + std::to_string(nonHtRefRateMbps) + "Mbps";
            std::cout << "Link " << std::to_string(i) <<" ControlRate: " << ctrlRateStr << " DataMode: " << dataModeStr << std::endl;
            wifi.SetRemoteStationManager(i, 
                                            "ns3::ConstantRateWifiManager",
                                            "DataMode", StringValue(dataModeStr),
                                            "ControlMode", StringValue(ctrlRateStr));
            if (i == 0) {
            dataModeStr = "HeMcs" + std::to_string(mcs[i]);
            nonHtRefRateMbps = HePhy::GetNonHtReferenceRate(mcs[i]) / 1e6;
            ctrlRateStr = "OfdmRate" + std::to_string(nonHtRefRateMbps) + "Mbps";
            std::cout << "2.4G OBSS: ControlRate: " << ctrlRateStr << " DataMode: " << dataModeStr << std::endl;
            wifi2.SetRemoteStationManager("ns3::ConstantRateWifiManager",
                                                "DataMode", StringValue(dataModeStr),
                                                "ControlMode", StringValue(ctrlRateStr));
            }
            else {
            dataModeStr = "HeMcs" + std::to_string(mcs[i]);
            nonHtRefRateMbps = HePhy::GetNonHtReferenceRate(mcs[i]) / 1e6;
            ctrlRateStr = "OfdmRate" + std::to_string(nonHtRefRateMbps) + "Mbps";
            std::cout << "5G OBSS: ControlRate: " << ctrlRateStr << " DataMode: " << dataModeStr << std::endl;
            wifi5.SetRemoteStationManager("ns3::ConstantRateWifiManager",
                                                "DataMode", StringValue(dataModeStr),
                                                "ControlMode", StringValue(ctrlRateStr));
            }
        }
    } else if (rateMode == "minstrel") {
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
    // phy.SetErrorRateModel("ns3::NistErrorRateModel");
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
    if (dlAckSeqType != "NO-OFDMA")
    {
        mac.SetMultiUserScheduler("ns3::RrMultiUserScheduler",
                                    "EnableUlOfdma",
                                    BooleanValue(enableUlOfdma),
                                    "EnableBsrp",
                                    BooleanValue(enableBsrp),
                                    "AccessReqInterval",
                                    TimeValue(accessReqInterval));
    }
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
    int16_t amsdu_maxn = 0;
    int16_t ampdu_maxn = 4;
    Config::Set("/NodeList/*/DeviceList/*/$ns3::WifiNetDevice/HeConfiguration/GuardInterval",
                    TimeValue(NanoSeconds(gi)));
    Config::Set("/NodeList/*/DeviceList/*/$ns3::WifiNetDevice/Mac/BE_MaxAmsduSize",
            UintegerValue(amsdu_maxn * (payloadSize + 150)));
    Config::Set("/NodeList/*/DeviceList/*/$ns3::WifiNetDevice/Mac/BE_MaxAmpduSize",
                    UintegerValue(ampdu_maxn * (payloadSize + 150)));
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
    positionAlloc->Add(Vector(20.0, 0.0, 0.0));
    positionAlloc->Add(Vector(0.0, 20.0, 0.0));
    positionAlloc->Add(Vector(0.0, 0.0, 0.0));
    positionAlloc->Add(Vector(20.0, 0.0, 0.0));
    positionAlloc->Add(Vector(0.0, 20.0, 0.0));
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
    ApplicationContainer ulserverApp;
    ApplicationContainer ulserverAppObss;
    // 1. DL 设置
    std::cout << "Link 0 maxload: " << EhtPhy::GetDataRate(mcs[0], bandwidth[0], NanoSeconds(gi), 1) / 1e6 << " Mbps" << std::endl;
    std::cout << "Link 1 maxload: " << EhtPhy::GetDataRate(mcs[1], bandwidth[1], NanoSeconds(gi), 1) / 1e6 << " Mbps" << std::endl;
    if (downlink) {
        const auto maxLoad = EhtPhy::GetDataRate(mcs[0], bandwidth[0], NanoSeconds(gi), 1) + EhtPhy::GetDataRate(mcs[1], bandwidth[1], NanoSeconds(gi), 1) ;
        if (udp)
        {
            // DL udp flow
            /* install server app for bss */
            uint16_t port = 9;
            UdpServerHelper server(port);
            dlserverApp = server.Install(mldNodes);
            seedNumber += server.AssignStreams(mldNodes, seedNumber);
            dlserverApp.Start(Seconds(0.0));
            dlserverApp.Stop(simulationTime + Seconds(1.0));

            /* install server app for obss */
            UdpServerHelper server2(port);
            dlserverAppObss2 = server2.Install(sldNodes2);
            seedNumber += server2.AssignStreams(sldNodes2, seedNumber);
            dlserverAppObss2.Start(Seconds(0.0));
            dlserverAppObss2.Stop(simulationTime + Seconds(1.0));

            UdpServerHelper server5(port);
            dlserverAppObss5 = server5.Install(sldNodes5);
            seedNumber += server5.AssignStreams(sldNodes5, seedNumber);
            dlserverAppObss5.Start(Seconds(0.0));
            dlserverAppObss5.Stop(simulationTime + Seconds(1.0));
            /* install clientapp for bss */
            const auto packetInterval = payloadSize * 8.0 / maxLoad;
            for (size_t i = 0; i < nStaMlds; i++) 
            {
                UdpClientHelper client(mldNodeInterface.GetAddress(i), port);
                client.SetAttribute("MaxPackets", UintegerValue(4294967295U));
                client.SetAttribute("Interval", TimeValue(Seconds(packetInterval)));
                client.SetAttribute("PacketSize", UintegerValue(payloadSize));
                ApplicationContainer clientApp = client.Install(apNodes.Get(0));
                seedNumber += client.AssignStreams(apNodes.Get(0), seedNumber);
                clientApp.Start(Seconds(2.0));
                clientApp.Stop(simulationTime + Seconds(1.0));
            }
            /* install clientapp for obss */
            const auto packetInterval2 = payloadSize * 8.0 / maxLoad;
            for (std::size_t i = 0; i < nStaSlds[0]; i++) 
            { 
                UdpClientHelper client(sldNodeInterface2.GetAddress(i), port);
                client.SetAttribute("MaxPackets", UintegerValue(4294967295U));
                client.SetAttribute("Interval", TimeValue(Seconds(packetInterval2)));
                client.SetAttribute("PacketSize", UintegerValue(payloadSize));
                ApplicationContainer clientApp2 = client.Install(apNodes.Get(1));
                seedNumber += client.AssignStreams(apNodes.Get(1), seedNumber);
                clientApp2.Start(Seconds(1.0));
                clientApp2.Stop(simulationTime + Seconds(1.0));
            }

            for (std::size_t i = 0; i < nStaSlds[1]; i++) 
            { 

                UdpClientHelper client(sldNodeInterface5.GetAddress(i), port);
                client.SetAttribute("MaxPackets", UintegerValue(4294967295U));
                client.SetAttribute("Interval", TimeValue(Seconds(packetInterval2)));
                client.SetAttribute("PacketSize", UintegerValue(payloadSize));
                ApplicationContainer clientApp5 = client.Install(apNodes.Get(2));
                seedNumber += client.AssignStreams(apNodes.Get(2), seedNumber);
                clientApp5.Start(Seconds(1.0));
                clientApp5.Stop(simulationTime + Seconds(1.0));
                // clientApp5.Get(0)->TraceConnectWithoutContext("Tx", MakeCallback(&NotifyAppTx));
            }
        }
        else
        {
            // DL tcp flow
            uint16_t port = 50000;
            Address localAddress(InetSocketAddress(Ipv4Address::GetAny(), port));
            PacketSinkHelper packetSinkHelper("ns3::TcpSocketFactory", localAddress);
            dlserverApp = packetSinkHelper.Install(mldNodes);
            seedNumber += packetSinkHelper.AssignStreams(mldNodes, seedNumber);
            dlserverApp.Start(Seconds(0.0));
            dlserverApp.Stop(simulationTime + Seconds(1.0));

            for (size_t i = 0; i < nStaMlds; i++)
            {
                OnOffHelper onoff("ns3::TcpSocketFactory", Ipv4Address::GetAny());
                onoff.SetAttribute("OnTime",
                                    StringValue("ns3::ConstantRandomVariable[Constant=1]"));
                onoff.SetAttribute("OffTime",
                                    StringValue("ns3::ConstantRandomVariable[Constant=0]"));
                onoff.SetAttribute("PacketSize", UintegerValue(payloadSize));
                onoff.SetAttribute("DataRate", DataRateValue(maxLoad));
                AddressValue remoteAddress(InetSocketAddress(mldNodeInterface.GetAddress(i), port));
                onoff.SetAttribute("Remote", remoteAddress);
                ApplicationContainer clientApp = onoff.Install(apNodes.Get(0));
                seedNumber += onoff.AssignStreams(apNodes.Get(0), seedNumber);

                clientApp.Start(Seconds(1.0));
                clientApp.Stop(simulationTime + Seconds(1.0));
            }

            for (size_t i = 0; i < nStaSlds[0]; i++)
            {
                OnOffHelper onoff("ns3::TcpSocketFactory", Ipv4Address::GetAny());
                onoff.SetAttribute("OnTime",
                                    StringValue("ns3::ConstantRandomVariable[Constant=1]"));
                onoff.SetAttribute("OffTime",
                                    StringValue("ns3::ConstantRandomVariable[Constant=0]"));
                onoff.SetAttribute("PacketSize", UintegerValue(payloadSize));
                onoff.SetAttribute("DataRate", DataRateValue(maxLoad));
                AddressValue remoteAddress(InetSocketAddress(sldNodeInterface2.GetAddress(i), port));
                onoff.SetAttribute("Remote", remoteAddress);
                ApplicationContainer clientApp = onoff.Install(apNodes.Get(1));
                seedNumber += onoff.AssignStreams(apNodes.Get(1), seedNumber);

                clientApp.Start(Seconds(1.0));
                clientApp.Stop(simulationTime + Seconds(1.0));
            }
            for (size_t i = 0; i < nStaSlds[1]; i++)
            {
                OnOffHelper onoff("ns3::TcpSocketFactory", Ipv4Address::GetAny());
                onoff.SetAttribute("OnTime",
                                    StringValue("ns3::ConstantRandomVariable[Constant=1]"));
                onoff.SetAttribute("OffTime",
                                    StringValue("ns3::ConstantRandomVariable[Constant=0]"));
                onoff.SetAttribute("PacketSize", UintegerValue(payloadSize));
                onoff.SetAttribute("DataRate", DataRateValue(maxLoad));
                AddressValue remoteAddress(InetSocketAddress(sldNodeInterface5.GetAddress(i), port));
                onoff.SetAttribute("Remote", remoteAddress);
                ApplicationContainer clientApp = onoff.Install(apNodes.Get(2));
                seedNumber += onoff.AssignStreams(apNodes.Get(2), seedNumber);

                clientApp.Start(Seconds(1.0));
                clientApp.Stop(simulationTime + Seconds(1.0));
            }
        }
    }

    // 2. UL 设置
    if (uplink) {
        const auto maxLoad = EhtPhy::GetDataRate(mcs[0], bandwidth[0], NanoSeconds(gi), 1) + EhtPhy::GetDataRate(mcs[1], bandwidth[1], NanoSeconds(gi), 1) ;
        if (udp)
        {   // UL udp flow
            /* install serverapp */
            uint16_t port = 9;
            UdpServerHelper server(port);
            ulserverApp = server.Install(apNodes);
            seedNumber += server.AssignStreams(apNodes, seedNumber);
            ulserverApp.Start(Seconds(0.0));
            ulserverApp.Stop(simulationTime + Seconds(1.0));

            /* install clientapp for bss */
            const auto packetInterval = payloadSize * 8.0 / maxLoad;
            for (std::size_t i = 0; i < nStaMlds; i++) 
            {
                UdpClientHelper client(apNodeInterface.GetAddress(0), port);
                client.SetAttribute("MaxPackets", UintegerValue(4294967295U));
                client.SetAttribute("Interval", TimeValue(Seconds(packetInterval)));
                client.SetAttribute("PacketSize", UintegerValue(payloadSize));
                ApplicationContainer clientApp = client.Install(mldNodes.Get(i));
                seedNumber += client.AssignStreams(mldNodes, seedNumber);
                
                clientApp.Start(Seconds(1.0));
                clientApp.Stop(simulationTime + Seconds(1.0));
            }
            /* install clientapp for obss */
            for (std::size_t i = 0; i < sldNodes2.GetN(); i++) 
            {
                UdpClientHelper client(apNodeInterface2.GetAddress(0), port);
                client.SetAttribute("MaxPackets", UintegerValue(4294967295U));
                client.SetAttribute("Interval", TimeValue(Seconds(packetInterval)));
                client.SetAttribute("PacketSize", UintegerValue(payloadSize));
                ApplicationContainer clientApp = client.Install(sldNodes2.Get(i));
                seedNumber += client.AssignStreams(mldNodes, seedNumber);
                clientApp.Start(Seconds(1.0));
                clientApp.Stop(simulationTime + Seconds(1.0));
            }
            for (std::size_t i = 0; i < sldNodes5.GetN(); i++) 
            {
                UdpClientHelper client(apNodeInterface5.GetAddress(0), port);
                client.SetAttribute("MaxPackets", UintegerValue(4294967295U));
                client.SetAttribute("Interval", TimeValue(Seconds(packetInterval)));
                client.SetAttribute("PacketSize", UintegerValue(payloadSize));
                ApplicationContainer clientApp = client.Install(sldNodes5.Get(i));
                seedNumber += client.AssignStreams(mldNodes, seedNumber);
                clientApp.Start(Seconds(1.0));
                clientApp.Stop(simulationTime + Seconds(1.0));
            }
        }
        else
        {
            // UL tcp flow
            uint16_t port = 50000;
            Address localAddress(InetSocketAddress(Ipv4Address::GetAny(), port));
            PacketSinkHelper packetSinkHelper("ns3::TcpSocketFactory", localAddress);
            ulserverApp = packetSinkHelper.Install(apNodes);
            seedNumber += packetSinkHelper.AssignStreams(apNodes, seedNumber);
            ulserverApp.Start(Seconds(0.0));
            ulserverApp.Stop(simulationTime + Seconds(1.0));
            /* install client for obss */
            for (std::size_t i = 0; i < nStaMlds; i++)
            {
                OnOffHelper onoff("ns3::TcpSocketFactory", Ipv4Address::GetAny());
                onoff.SetAttribute("OnTime",
                                    StringValue("ns3::ConstantRandomVariable[Constant=1]"));
                onoff.SetAttribute("OffTime",
                                    StringValue("ns3::ConstantRandomVariable[Constant=0]"));
                onoff.SetAttribute("PacketSize", UintegerValue(payloadSize));
                onoff.SetAttribute("DataRate", DataRateValue(maxLoad));
                AddressValue remoteAddress(InetSocketAddress(apNodeInterface.GetAddress(0), port));
                onoff.SetAttribute("Remote", remoteAddress);
                ApplicationContainer clientApp = onoff.Install(mldNodes.Get(i));
                seedNumber += onoff.AssignStreams(mldNodes.Get(i), seedNumber);

                clientApp.Start(Seconds(1.0));
                clientApp.Stop(simulationTime + Seconds(1.0));
            }
            /* install client for obss */
            for (std::size_t i = 0; i < nStaSlds[0]; i++)
            {
                OnOffHelper onoff("ns3::TcpSocketFactory", Ipv4Address::GetAny());
                onoff.SetAttribute("OnTime",
                                    StringValue("ns3::ConstantRandomVariable[Constant=1]"));
                onoff.SetAttribute("OffTime",
                                    StringValue("ns3::ConstantRandomVariable[Constant=0]"));
                onoff.SetAttribute("PacketSize", UintegerValue(payloadSize));
                onoff.SetAttribute("DataRate", DataRateValue(maxLoad));
                AddressValue remoteAddress(InetSocketAddress(apNodeInterface2.GetAddress(0), port));
                onoff.SetAttribute("Remote", remoteAddress);
                ApplicationContainer clientApp = onoff.Install(sldNodes2.Get(i));
                seedNumber += onoff.AssignStreams(sldNodes2.Get(i), seedNumber);
                clientApp.Start(Seconds(1.0));
                clientApp.Stop(simulationTime + Seconds(1.0));
            }
            for (std::size_t i = 0; i < nStaSlds[1]; i++)
            {
                OnOffHelper onoff("ns3::TcpSocketFactory", Ipv4Address::GetAny());
                onoff.SetAttribute("OnTime",
                                    StringValue("ns3::ConstantRandomVariable[Constant=1]"));
                onoff.SetAttribute("OffTime",
                                    StringValue("ns3::ConstantRandomVariable[Constant=0]"));
                onoff.SetAttribute("PacketSize", UintegerValue(payloadSize));
                onoff.SetAttribute("DataRate", DataRateValue(maxLoad));
                AddressValue remoteAddress(InetSocketAddress(apNodeInterface5.GetAddress(0), port));
                onoff.SetAttribute("Remote", remoteAddress);
                ApplicationContainer clientApp = onoff.Install(sldNodes5.Get(i));
                seedNumber += onoff.AssignStreams(sldNodes5.Get(i), seedNumber);
                clientApp.Start(Seconds(1.0));
                clientApp.Stop(simulationTime + Seconds(1.0));
            }
        }
    }

     // cumulative number of bytes received by each server application
    std::vector<uint64_t> cumulRxBytes(nStaMlds, 0);

    if (tputInterval.IsStrictlyPositive())
    {
        Simulator::Schedule(Seconds(1) + tputInterval,
                            &PrintIntermediateTput,
                            cumulRxBytes,
                            udp,
                            ulserverApp,
                            payloadSize,
                            tputInterval,
                            simulationTime + Seconds(1.0));
    }

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

    /* 数据统计模块 */
    WifiTxStatsHelper ApStats;
    ApStats.Enable(devices);
    ApStats.Start(Seconds(0));
    ApStats.Stop(simulationTime + Seconds(1.0));

    // AsciiTraceHelper asciiTrace;
    // phy.EnableAsciiAll(asciiTrace.CreateFileStream("single-bss-coex.tr"));
    phy.EnablePcap("ap0-trace", apDev.Get(0));
    phySld2.EnablePcap("ap1-trace", apDev.Get(1));
    phySld5.EnablePcap("ap2-trace", apDev.Get(2));
    phy.EnablePcap("mld-trace", mldDev.Get(0));
    phySld2.EnablePcap("sld2-trace", sldDev2.Get(0));
    phySld5.EnablePcap("sld5-trace", sldDev5.Get(0));
    Simulator::Stop(simulationTime + Seconds(1.2));
    Simulator::Run();
    std::vector<uint64_t> ulCumulRxBytes(nStaSlds[0], 0);
    std::vector<uint64_t> dlCumulRxBytes(nStaSlds[0], 0);
    ulCumulRxBytes = GetRxBytes(udp, ulserverApp, payloadSize);
    auto ulrxBytes = std::accumulate(ulCumulRxBytes.cbegin(), ulCumulRxBytes.cend(), 0.0);
    auto ulthroughput = (ulrxBytes * 8) / simulationTime.GetMicroSeconds(); // Mbit/s

    dlCumulRxBytes = GetRxBytes(udp, dlserverApp, payloadSize);
    auto dlrxBytes = std::accumulate(dlCumulRxBytes.cbegin(), dlCumulRxBytes.cend(), 0.0);
    auto dlthroughput = ((double)dlrxBytes * 8) / simulationTime.GetMicroSeconds(); // Mbit/s
    std::cout << "UL Throughput: \t\t\t" << ulthroughput << " Mbit/s" << std::endl;
    std::cout << "DL Throughput: \t\t\t" << dlthroughput << " Mbit/s" << std::endl;

    auto results = ApStats.GetFinalStatistics();
    std::cout << "from, \t to, \t throughput on link 0(mbps),\t throughput on link 1(mbps), \t mean access delay on link 0(ms),\t mean access delay on link 1(ms)" << std::endl;
    auto totalNodeNum = devices.GetN();
    for (uint32_t i = 0; i < totalNodeNum; i++) {
        for (uint32_t j = 0; j < totalNodeNum; j++) {
            if (i == j) continue;
            uint32_t from = i;
            uint32_t to = j;
            uint64_t rxBytes0 = results.m_numSuccessPerNodePairLink[{from, to}][0] * payloadSize;
            uint64_t rxBytes1 = results.m_numSuccessPerNodePairLink[{from, to}][1] * payloadSize;
            // std::cout << (double)(rxBytes0 + rxBytes1) * 8 / simulationTime.GetMicroSeconds() << std::endl;
            // results.m_numFinalFailedPerNodePair[{from, to}]
            std::cout << (int)from << ", " << (int)to << ", " << double(rxBytes0) * 8 / (simulationTime.GetSeconds()) / 1e6 << ", " << double(rxBytes1) * 8 / (simulationTime.GetSeconds()) / 1e6 << ", "
            << results.m_meanAccessDelayPerNodePairLink[{from, to}][0] << ", " << results.m_meanAccessDelayPerNodePairLink[{from, to}][1] << std::endl;
        }
    }
    Simulator::Destroy();
    return 0;
}