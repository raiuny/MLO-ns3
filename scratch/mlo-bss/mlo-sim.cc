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

NS_LOG_COMPONENT_DEFINE("mlo-bss");
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
    int gi = 800;
    uint16_t mpduBufferSize{64};
    Time simulationTime{"10s"};
    std::string dlAckSeqType{"NO-OFDMA"};
    meter_u distance{1.0};
    std::size_t nStaMlds{1};
    std::vector<std::size_t> nStaSlds{0, 0};
    bool enableUlOfdma{false};
    bool enableBsrp{false};
    uint32_t payloadSize = 700; // must fit in the max TX duration when transmitting at MCS 0 over an RU of 26 tones
    Time tputInterval{0}; // interval for detailed throughput measurement
    double minExpectedThroughput{0};
    double maxExpectedThroughput{0};
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
    apNodes.Create(1);
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
    std::vector<int> mcs{0, 6};
    std::vector<int> bandwidth{20, 40};
    std::string dataModeStr = "EhtMcs" + std::to_string(mcs[0]);
    uint64_t nonHtRefRateMbps;
    std::string ctrlRateStr; 
    for (uint8_t i = 0; i < nLinks; ++i) {
        dataModeStr = "EhtMcs" + std::to_string(mcs[i]);
        nonHtRefRateMbps = EhtPhy::GetNonHtReferenceRate(mcs[i]) / 1e6;
        ctrlRateStr = "OfdmRate" + std::to_string(nonHtRefRateMbps) + "Mbps";
        std::cout << "Link " << std::to_string(i) <<" ControlRate: " << ctrlRateStr << " DataMode: " << dataModeStr << std::endl;
        wifi.SetRemoteStationManager(i, 
                                        "ns3::ConstantRateWifiManager",
                                        "DataMode", StringValue(dataModeStr),
                                        "ControlMode", StringValue(ctrlRateStr));
        if (i == 0) wifi2.SetRemoteStationManager("ns3::ConstantRateWifiManager",
                                            "DataMode", StringValue(dataModeStr),
                                            "ControlMode", StringValue(ctrlRateStr));
        else wifi5.SetRemoteStationManager("ns3::ConstantRateWifiManager",
                                            "DataMode", StringValue(dataModeStr),
                                            "ControlMode", StringValue(ctrlRateStr));
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
    SpectrumWifiPhyHelper phySld2;
    phySld2.SetPcapDataLinkType(WifiPhyHelper::DLT_IEEE802_11_RADIO);
    phySld2.SetErrorRateModel("ns3::TableBasedErrorRateModel");
    phySld2.Set("TxPowerStart", DoubleValue(txPower));
    phySld2.Set("TxPowerEnd", DoubleValue(txPower));
    phySld2.AddChannel(spectrumChannel_2, WIFI_SPECTRUM_2_4_GHZ);
    phySld2.Set("ChannelSettings", StringValue("{0, " + std::to_string(bandwidth[0]) + ", BAND_2_4GHZ, 0}"));

    SpectrumWifiPhyHelper phySld5;
    phySld5.SetPcapDataLinkType(WifiPhyHelper::DLT_IEEE802_11_RADIO);
    phySld5.SetErrorRateModel("ns3::TableBasedErrorRateModel");
    phySld5.Set("TxPowerStart", DoubleValue(txPower));
    phySld5.Set("TxPowerEnd", DoubleValue(txPower));
    phySld5.AddChannel(spectrumChannel_5, WIFI_SPECTRUM_5_GHZ);
    phySld5.Set("ChannelSettings", StringValue("{0, " + std::to_string(bandwidth[1]) + ", BAND_5GHZ, 0}"));

    /* MAC Configuration */
    WifiMacHelper mac;
    Ssid bssSsid = Ssid("BSS-SLD-MLD-COEX");
    // 1. STA MAC 设置
    mac.SetType("ns3::StaWifiMac", 
                "MaxMissedBeacons", UintegerValue(std::numeric_limits<uint32_t>::max()), 
                "Ssid", SsidValue(bssSsid));
    mldDev = wifi.Install(phy, mac, mldNodes.Get(0));
    sldDev2 = wifi2.Install(phySld2, mac, sldNodes2);
    sldDev5 = wifi5.Install(phySld5, mac, sldNodes5);

    // print mac address of mld 
    Ptr<WifiMac> mac1 = DynamicCast<WifiNetDevice>(mldDev.Get(0))->GetMac();
    std::cout << "STA-MAC: " << mldDev.Get(0)->GetAddress() << std::endl;
    for (uint8_t i = 0; i < nLinks; ++i) {
        auto fem = mac1->GetFrameExchangeManager(i);
        std::cout << "mldDevice " << "linkId " << std::to_string(i) << " mac address: " << fem->GetAddress() << std::endl;
    }
    
    // print mac address of slds
    Ptr<WifiMac> mac_sld;
    for (int i = 0; i < 2; ++i) { 
        if (nStaSlds[i])
            std::cout << "sldDevice " << "linkId " << std::to_string(i) << ":" << std::endl;
        for (auto j = 0; j < nStaSlds[i]; ++j) {
            if (i == 0) mac_sld = DynamicCast<WifiNetDevice>(sldDev2.Get(j))->GetMac();
            else mac_sld = DynamicCast<WifiNetDevice>(sldDev5.Get(j))->GetMac();
            auto fem = mac_sld->GetFrameExchangeManager();
            std::cout << "  SLD-" << j+1 << ": " << fem->GetAddress() << std::endl;
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
    apDev = wifi.Install(phy, mac, apNodes);
    
    // print mac address of ap
    std::cout << "AP-MAC: " << apDev.Get(0)->GetAddress()<< std::endl;
    Ptr<WifiMac> mac2 = DynamicCast<WifiNetDevice>(apDev.Get(0))->GetMac();
    for (uint8_t i = 0; i < nLinks; ++i) {
        auto fem = mac2->GetFrameExchangeManager(i);
        std::cout << "apDevice " << "linkId " << std::to_string(i) << " mac address: " << fem->GetAddress() << std::endl;
    }

    NetDeviceContainer devices;
    devices.Add(apDev);
    devices.Add(mldDev);
    devices.Add(sldDev2);
    devices.Add(sldDev5);
    wifi.AssignStreams(devices, seedNumber);

    // Set guard interval, MPDU buffer size, MaxAmsduSize, MaxAmpduSize
    int16_t amsdu_maxn = 1;
    int16_t ampdu_maxn = 1;
    Config::Set("/NodeList/*/DeviceList/*/$ns3::WifiNetDevice/HeConfiguration/GuardInterval",
                    TimeValue(NanoSeconds(gi)));
    Config::Set("/NodeList/*/DeviceList/*/$ns3::WifiNetDevice/Mac/BE_MaxAmsduSize",
            UintegerValue(amsdu_maxn * (payloadSize + 150)));
    Config::Set("/NodeList/*/DeviceList/*/$ns3::WifiNetDevice/Mac/BE_MaxAmpduSize",
                    UintegerValue(ampdu_maxn * amsdu_maxn * (payloadSize + 150)));
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
    for (uint8_t i = 0; i < nStaMlds + nStaSlds[0] + nStaSlds[1]; ++i) {
        if (i == 0) positionAlloc->Add(Vector(distance, 0.0, 0.0));
        else positionAlloc->Add(Vector(Rng->GetValue(), Rng->GetValue() ,0));
    }
    mobility.SetPositionAllocator(positionAlloc);
    NodeContainer allNodes(apNodes, mldNodes, sldNodes2, sldNodes5);
    mobility.Install(allNodes);

    /* Internet stack*/
    InternetStackHelper stack;
    stack.Install(allNodes);
    seedNumber += stack.AssignStreams(allNodes, seedNumber);

    Ipv4AddressHelper address;
    address.SetBase("10.0.0.0", "255.255.255.0");
    Ipv4InterfaceContainer apNodeInterface = address.Assign(apDev);
    Ipv4InterfaceContainer mldNodeInterface = address.Assign(mldDev);
    Ipv4InterfaceContainer sldNodeInterface2 = address.Assign(sldDev2);
    Ipv4InterfaceContainer sldNodeInterface5 = address.Assign(sldDev5);
    // print Ip address of devices
    std::cout << "AP-IP: " << apNodeInterface.GetAddress(0) << std::endl;
    std::cout << "STA-IP: " << std::endl;
    for (int i = 0; i < nStaMlds; ++i) {
        std::cout << "  STA-MLD-" << std::to_string(i) + ": " << mldNodeInterface.GetAddress(i) << std::endl;
    }
    for (int i = 0; i < nStaSlds[0];  ++i) {
        std::cout << "  Link 0 STA-SLD-" << std::to_string(i) + ": " << sldNodeInterface2.GetAddress(i) << std::endl;
    }
    for (int i = 0; i < nStaSlds[1];  ++i) {
        std::cout << "  Link 1 STA-SLD-" << std::to_string(i) + ": " << sldNodeInterface5.GetAddress(i) << std::endl;
    }
    /* Setting applications */
    ApplicationContainer dlserverApp;
    ApplicationContainer ulserverApp;
    // 1. DL 设置
    std::cout << "Link 0 maxload: " << EhtPhy::GetDataRate(mcs[0], bandwidth[0], NanoSeconds(gi), 1) / 1e6 << " Mbps" << std::endl;
    std::cout << "Link 1 maxload: " << EhtPhy::GetDataRate(mcs[1], bandwidth[1], NanoSeconds(gi), 1) / 1e6 << " Mbps" << std::endl;
    if (downlink) {
        const auto maxLoad = EhtPhy::GetDataRate(mcs[0], bandwidth[0], NanoSeconds(gi), 1) + EhtPhy::GetDataRate(mcs[1], bandwidth[1], NanoSeconds(gi), 1) ;
        if (udp)
        {
            // DL udp flow
            /* install serverapp */
            uint16_t port = 9;
            UdpServerHelper server(port);
            dlserverApp = server.Install(mldNodes);
            seedNumber += server.AssignStreams(mldNodes, seedNumber);
            dlserverApp.Start(Seconds(0.0));
            dlserverApp.Stop(simulationTime + Seconds(1.2));

            /* install clientapp */
            const auto packetInterval = payloadSize * 8.0 / maxLoad;
            for (std::size_t i = 0; i < nStaMlds; i++) 
            {
                UdpClientHelper client(mldNodeInterface.GetAddress(i), port);
                client.SetAttribute("MaxPackets", UintegerValue(4294967295U));
                client.SetAttribute("Interval", TimeValue(Seconds(packetInterval)));
                client.SetAttribute("PacketSize", UintegerValue(payloadSize));
                ApplicationContainer clientApp = client.Install(apNodes.Get(0));
                seedNumber += client.AssignStreams(apNodes, seedNumber);

                clientApp.Start(Seconds(1.0));
                clientApp.Stop(simulationTime + Seconds(1.0));
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
            dlserverApp.Stop(simulationTime + Seconds(1.1));

            for (std::size_t i = 0; i < nStaMlds; i++)
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
                seedNumber += onoff.AssignStreams(apNodes, seedNumber);

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

            /* install clientapp */
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
                seedNumber += onoff.AssignStreams(mldNodes, seedNumber);

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
    ApStats.Stop(simulationTime + Seconds(1.2));

    // AsciiTraceHelper asciiTrace;
    // phy.EnableAsciiAll(asciiTrace.CreateFileStream("single-bss-coex.tr"));
    phy.EnablePcap("ap-trace", apDev.Get(0));
    phy.EnablePcap("mld-trace", mldDev.Get(0));
    // phy.EnablePcap("sld-2.4-trace", sldDev2);
    // phy.EnablePcap("sld-5-trace", sldDev5);
    Simulator::Stop(simulationTime + Seconds(1.2));
    Simulator::Run();
    auto tolerance = 0.10;
    std::vector<uint64_t> ulCumulRxBytes(nStaMlds, 0);
    std::vector<uint64_t> dlCumulRxBytes(nStaMlds, 0);
    ulCumulRxBytes = GetRxBytes(udp, ulserverApp, payloadSize);
    auto ulrxBytes = std::accumulate(ulCumulRxBytes.cbegin(), ulCumulRxBytes.cend(), 0.0);
    auto ulthroughput = (ulrxBytes * 8) / simulationTime.GetMicroSeconds(); // Mbit/s

    dlCumulRxBytes = GetRxBytes(udp, dlserverApp, payloadSize);
    auto dlrxBytes = std::accumulate(dlCumulRxBytes.cbegin(), dlCumulRxBytes.cend(), 0.0);
    auto dlthroughput = ((double)dlrxBytes * 8) / simulationTime.GetMicroSeconds(); // Mbit/s
    std::cout << "UL Throughput: \t\t\t" << ulthroughput << " Mbit/s" << std::endl;
    std::cout << "DL Throughput: \t\t\t" << dlthroughput << " Mbit/s" << std::endl;

    auto results = ApStats.GetFinalStatistics();
    std::cout << "from, \t to, \t throughput on link 0(mbps),\t throughput on link 1(mbps), SuccessProbability on link 0, SuccessProbability on link 1, \t mean access delay on link 0(ms),\t mean access delay on link 1(ms)" << std::endl;
    uint32_t from  = 0;
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