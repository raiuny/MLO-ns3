#include "ns3/core-module.h"
#include "ns3/network-module.h"
#include "ns3/internet-module.h"
#include "ns3/wifi-module.h"
#include "ns3/mobility-module.h"
#include "ns3/applications-module.h"
#include "ns3/config-store-module.h"
#include "ns3/log.h"

using namespace ns3;

NS_LOG_COMPONENT_DEFINE("Wifi6SimpleExample");

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

int main(int argc, char *argv[])
{
    // 启用日志组件
    LogComponentEnable("Wifi6SimpleExample", LOG_LEVEL_INFO);

    // 创建节点
    NodeContainer apNode;
    NodeContainer staNode;
    apNode.Create(1);
    staNode.Create(1);

    // 创建无线设备
    WifiHelper wifi;
    wifi.SetStandard(WIFI_STANDARD_80211ax);  // 使用 Wi-Fi 6 (802.11ax) 标准

    SpectrumWifiPhyHelper phy(1); 
    phy.SetPcapDataLinkType(YansWifiPhyHelper::DLT_IEEE802_11_RADIO);

    // 设置频段为 2.4 GHz
    double txPower = 16; 
    Ptr<MultiModelSpectrumChannel> spectrumChannel_2 = CreateObject<MultiModelSpectrumChannel>();
    Ptr<LogDistancePropagationLossModel> lossModel_2 = CreateObject<LogDistancePropagationLossModel>();
    lossModel_2->SetAttribute("Exponent", DoubleValue(2.0));
    lossModel_2->SetAttribute("ReferenceDistance", DoubleValue(1.0));
    lossModel_2->SetAttribute("ReferenceLoss", DoubleValue(40.046));
    spectrumChannel_2->AddPropagationLossModel(lossModel_2);
    phy.AddChannel(spectrumChannel_2, WIFI_SPECTRUM_2_4_GHZ);
    phy.Set("ChannelSettings", StringValue("{0, 20, BAND_2_4GHZ, 0}"));
    phy.Set("TxPowerStart", DoubleValue(txPower));
    phy.Set("TxPowerEnd", DoubleValue(txPower));
    
    // 配置物理层参数
    // phy.SetErrorRateModel("ns3::NistErrorRateModel");
    phy.SetErrorRateModel("ns3::TableBasedErrorRateModel");


    // 配置 MAC 层参数
    WifiMacHelper mac;
    Ssid ssid = Ssid("wifi-6-ssid");
    mac.SetType("ns3::StaWifiMac",
                "Ssid", SsidValue(ssid),
                "ActiveProbing", BooleanValue(false));

    NetDeviceContainer staDevices = wifi.Install(phy, mac, staNode);

    mac.SetType("ns3::ApWifiMac",
                "Ssid", SsidValue(ssid));

    NetDeviceContainer apDevices = wifi.Install(phy, mac, apNode);

    // 配置移动模型
    MobilityHelper mobility;
    Ptr<ListPositionAllocator> positionAlloc = CreateObject<ListPositionAllocator>();
    positionAlloc->Add(Vector(0.0, 0.0, 0.0));  // AP 位置
    positionAlloc->Add(Vector(5.0, 0.0, 0.0));  // STA 位置
    mobility.SetPositionAllocator(positionAlloc);
    mobility.SetMobilityModel("ns3::ConstantPositionMobilityModel");
    mobility.Install(apNode);
    mobility.Install(staNode);

    // 配置互联网协议栈
    InternetStackHelper stack;
    stack.Install(apNode);
    stack.Install(staNode);

    Ipv4AddressHelper address;
    address.SetBase("10.1.1.0", "255.255.255.0");
    Ipv4InterfaceContainer apInterfaces = address.Assign(apDevices);
    Ipv4InterfaceContainer staInterfaces = address.Assign(staDevices);

    // 配置应用程序
    UdpServerHelper Server(9);
    ApplicationContainer serverApps = Server.Install(staNode);
    serverApps.Start(Seconds(1.0));
    serverApps.Stop(Seconds(10.0));

    UdpClientHelper Client(staInterfaces.GetAddress(0), 9);
    Client.SetAttribute("MaxPackets", UintegerValue(10));
    Client.SetAttribute("Interval", TimeValue(Seconds(1.0)));
    Client.SetAttribute("PacketSize", UintegerValue(1024));

    ApplicationContainer clientApps = Client.Install(apNode);
    clientApps.Start(Seconds(2.0));
    clientApps.Stop(Seconds(10.0));

    // 连接发送和接收回调
    clientApps.Get(0)->TraceConnectWithoutContext("Tx", MakeCallback(&NotifyAppTx));
    serverApps.Get(0)->TraceConnectWithoutContext("Rx", MakeCallback(&NotifyAppRx));
    WifiTxStatsHelper ApStats;
    NetDeviceContainer devices;
    devices.Add(staDevices);
    devices.Add(apDevices);
    ApStats.Enable(devices);
    ApStats.Start(Seconds(0.1));
    ApStats.Stop(Seconds(10.0));
    
    // 启动仿真
    Simulator::Stop(Seconds(10.0));
    Simulator::Run();
    auto rxBytes = serverApps.Get(0)->GetObject<UdpServer>()->GetReceived() * 1024;
    double totalThroughputMbps = (double)(rxBytes) * 8 / 10 / 1e6;
    std::cout << "Total Throughput: " << totalThroughputMbps << " Mbps" << std::endl;
    std::cout << "from, \t to, \t throughput(mbps),\t Mean Access Delay" << std::endl;
    int8_t from = 0;
    int8_t to = 1;
    auto results = ApStats.GetFinalStatistics();

    rxBytes = results.m_numSuccessPerNodePairLink[{from, to}][0] * 1024;
    std::cout << "throughput: " << rxBytes * 8 / 10 / 1e6 << std::endl;
    std::cout << "Access Delay: " << results.m_meanAccessDelayPerNodePairLink[{from, to}][0] << std::endl;
    from = 1, to = 0;
    rxBytes = results.m_numSuccessPerNodePairLink[{from, to}][0] * 1024;
    std::cout << "throughput: " << rxBytes * 8 / 10 / 1e6 << std::endl;
    std::cout << "Access Delay: " << results.m_meanAccessDelayPerNodePairLink[{from, to}][0] << std::endl;
    Simulator::Destroy();
    return 0;
}