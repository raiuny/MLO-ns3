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
#include "ns3/ap-wifi-mac.h"
#include "ns3/attribute-container.h"
#include "ns3/command-line.h"
#include "ns3/config.h"
#include "ns3/constant-rate-wifi-manager.h"
#include "ns3/eht-configuration.h"
#include "ns3/eht-phy.h"
#include "ns3/flow-monitor-module.h"
#include "ns3/frame-exchange-manager.h"
#include "ns3/internet-stack-helper.h"
#include "ns3/ipv4-address-helper.h"
#include "ns3/log.h"
#include "ns3/mobility-helper.h"
#include "ns3/multi-model-spectrum-channel.h"
#include "ns3/on-off-helper.h"
#include "ns3/packet-sink-helper.h"
#include "ns3/packet-sink.h"
#include "ns3/packet-socket-client.h"
#include "ns3/packet-socket-helper.h"
#include "ns3/packet-socket-server.h"
#include "ns3/qos-utils.h"
#include "ns3/rng-seed-manager.h"
#include "ns3/spectrum-wifi-helper.h"
#include "ns3/sta-wifi-mac.h"
#include "ns3/string.h"
#include "ns3/tcp-socket.h"
#include "ns3/udp-client-server-helper.h"
#include "ns3/udp-server.h"
#include "ns3/uinteger.h"
#include "ns3/wifi-mac-queue.h"
#include "ns3/wifi-net-device.h"
#include "ns3/wifi-phy-common.h"
#include "ns3/wifi-phy-rx-trace-helper.h"
#include "ns3/wifi-phy.h"
#include "ns3/wifi-tx-stats-helper.h"
#include "ns3/wifi-utils.h"
#include "ns3/yans-wifi-helper.h"

#include <fstream>
#include <iomanip>
#include <iostream>
#include <ostream>
#include <sstream>
#include <sys/stat.h>
#include <vector>
#define PI 3.1415926535

using namespace ns3;
std::string outputfile("PPDU_no_interference_no_newarch");

NS_LOG_COMPONENT_DEFINE("mlo-obss-dl-udp");

struct Stats
{
    double throughput;
    double link1Pct;
    std::vector<double> thpt; // 两条链路分别
    std::vector<double> p;
    std::vector<double> occ;
    std::vector<double> datarate;
    std::vector<double> blocktimerate;
    std::vector<double> blockrate; // ？
    std::vector<uint32_t> blockCnt;
    std::vector<uint32_t> blockCnt_tr; // ？
    std::vector<uint32_t> txopNum;
    std::vector<uint64_t> txopTime;
    std::unordered_map<std::string, std::vector<uint32_t>> params;
    std::vector<uint32_t> maxAmpduLength;
    std::vector<uint32_t> meanAmpduLength;

    Stats(double tp,
          double pct,
          std::vector<double> t,
          std::vector<double> pr,
          std::vector<double> oc,
          std::vector<double> dr,
          std::vector<double> btr,
          std::vector<double> br,
          std::vector<uint32_t> bc,
          std::vector<uint32_t> bc2,
          std::vector<uint32_t> tn,
          std::vector<uint64_t> tt,
          std::unordered_map<std::string, std::vector<uint32_t>> pm,
          std::vector<uint32_t> maxl,
          std::vector<uint32_t> meanl)
        : throughput(tp),
          link1Pct(pct),
          thpt(std::move(t)),
          p(std::move(pr)),
          occ(std::move(oc)),
          datarate(std::move(dr)),
          blocktimerate(std::move(btr)),
          blockrate(std::move(br)),
          blockCnt(std::move(bc)),
          blockCnt_tr(std::move(bc2)),
          txopNum(std::move(tn)),
          txopTime(std::move(tt)),
          params(std::move(pm)),
          maxAmpduLength(std::move(maxl)),
          meanAmpduLength(std::move(meanl))
    {
    }
};

Stats* res = nullptr;
double dlthroughput = 0.0;

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

// 需要这个来记录参数
void
SaveParams(std::unordered_map<std::string, std::vector<uint32_t>> pm,
           double link1Pct,
           std::vector<double> thpt,
           std::vector<double> p,
           std::vector<double> occ,
           std::vector<double> datarate,
           std::vector<double> blocktimerate,
           std::vector<double> blockrate,
           std::vector<uint32_t> blockCnt,
           std::vector<uint32_t> blockCnt_tr,
           std::vector<uint64_t> txopTime,
           std::vector<uint32_t> txopNum,
           std::vector<uint32_t> maxAmpduLength,
           std::vector<uint32_t> meanAmpduLength)
{
    res = new Stats{0.0,
                    link1Pct,
                    thpt,
                    p,
                    occ,
                    datarate,
                    blocktimerate,
                    blockrate,
                    blockCnt,
                    blockCnt_tr,
                    txopNum,
                    txopTime,
                    pm,
                    maxAmpduLength,
                    meanAmpduLength};
    auto& params = pm;
    std::cout
        << "No, Mode, CWmin1, CWmax1, CWmin2, CWmax2, Aifsn1, Aifsn2, TxopLimit1, TxopLimit2, "
           "RTS_CTS1, RTS_CTS2, MaxSlrc1, "
           "MaxSsrc1, MaxSlrc2, MaxSsrc2, RedundancyThreshold1, RedundancyThreshold2, "
           "RedundancyFixedNumber1, "
           "RedundancyFixedNumber2, Link1Pct, BlockCnt1, BlockCnt2, BlockCnt1_True, BlockCnt2_True, "
           "TxopTime1(us), TxopTime2(us), TxopCnt1, TxopCnt2, MaxAmpduLength1, MaxAmpduLength2, "
           "p1, p2, Occupancy Rate 1, Occupancy Rate 2, datarate1, datarate2, Throughput(Mbps)"
        << std::endl;
    std::cout << params["No"][0] << ", " << 2 << ", " << params["CWmins"][0] << ", "
              << params["CWmaxs"][0] << ", " << params["CWmins"][1] << ", " << params["CWmaxs"][1]
              << ", " << params["Aifsns"][0] << ", " << params["Aifsns"][1] << ", "
              << params["TxopLimits"][0] << ", " << params["TxopLimits"][1] << ", "
              << params["RTS_CTS"][0] << ", " << params["RTS_CTS"][1] << ", "
              << params["MaxSlrcs"][0] << ", " << params["MaxSsrcs"][0] << ", "
              << params["MaxSlrcs"][1] << ", " << params["MaxSsrcs"][1] << ", "
              << params["RedundancyThresholds"][0] << ", " << params["RedundancyThresholds"][1]
              << ", " << params["RedundancyFixedNumbers"][0] << ", "
              << params["RedundancyFixedNumbers"][1] << ", " << res->link1Pct << "," << res->blockCnt[0] << ", "
              << res->blockCnt[1] << ", " << res->blockCnt_tr[0] << ", " << res->blockCnt_tr[1] << ", "
              << res->txopTime[0] << ", " << res->txopTime[1] << ", " << res->txopNum[0] << ", "
              << res->txopNum[1] << ", " << res->maxAmpduLength[0] << ", " << res->maxAmpduLength[1]
              << ", " << res->p[0] << ", " << res->p[1] << ", " << res->occ[0] << ", " << res->occ[1]
              << ", " << res->datarate[0] << ", " << res->datarate[1] << ", " << res->throughput
              << std::endl;

}


void
NotifyPpduTxDuration(Ptr<const WifiPpdu> ppdu, Time duration, uint8_t linkid)
{
    if (!ppdu->GetPsdu()->GetHeader(0).IsQosData())
        return;
    // std::cout << "NotifyPpduTxDuration: " << Simulator::Now().GetSeconds() << std::endl;
    Ptr<const WifiPsdu> psdu = ppdu->GetPsdu();
    if (psdu->IsAggregate())
    {
        uint32_t nmpdus = psdu->GetNMpdus();
        if (Simulator::Now().GetSeconds() < 4 && Simulator::Now().GetSeconds() > 3)
        {
            std::fstream file;
            file.open(outputfile, std::ios::out | std::ios::app);
            if (nmpdus != 0)
                file << "OBSS" << "," << Simulator::Now().GetMicroSeconds() << ","
                     << Simulator::Now().GetMicroSeconds() + duration.GetMicroSeconds() << ","
                     << nmpdus << std::endl;
            file.close();
        }
    }
}

void
NotifyPpduTxDurationmld(Ptr<const WifiPpdu> ppdu, Time duration, uint8_t linkid)
{
    if (!ppdu->GetPsdu()->GetHeader(0).IsQosData())
        return;
    Ptr<const WifiPsdu> psdu = ppdu->GetPsdu();
    if (psdu->IsAggregate())
    {
        uint32_t nmpdus = psdu->GetNMpdus();
        if (Simulator::Now().GetSeconds() < 4 && Simulator::Now().GetSeconds() > 3) //change
        {
            std::fstream file;
            file.open(outputfile, std::ios::out | std::ios::app);
            if (nmpdus != 0)
                file << uint32_t(linkid) << "," << Simulator::Now().GetMicroSeconds() << ","
                     << Simulator::Now().GetMicroSeconds() + duration.GetMicroSeconds() << ","
                     << nmpdus << std::endl;
            file.close();
        }
    }
}

void
SavetxopTime(std::unordered_map<uint8_t, std::vector<std::pair<uint64_t, uint64_t>>> txoplist,
             std::unordered_map<uint8_t, std::vector<uint32_t>> txopnums)
{
    std::fstream file;
    file.open("txop" + outputfile, std::ios::out | std::ios::app);
    for (auto it = txoplist.begin(); it != txoplist.end(); it++)
    {
        uint8_t linkid = it->first;
        auto& txopTime = it->second;
        // auto& txopNum = txopnums[linkid];
        for (size_t i = 0; i < txopTime.size(); i++)
        {
            file << (uint32_t)linkid << "," << txopTime[i].first << "," << txopTime[i].second
                 << std::endl;
        }
    }
    file.close();
}

int
main(int argc, char* argv[])
{
    double link1Pct{0.004};
    uint32_t mcs1 = 6;
    uint32_t mcs2 = 11;
    uint32_t bw1 = 20;
    uint32_t bw2 = 160;
    double r1 = 0.1;
    double r2 = 1;
    std::uint32_t sceneId = 0;
    uint32_t ch1 = 0;
    uint32_t ch2 = 0;
    std::string rateCtrl{"constant"};
    // std::string rateCtrl{"minstrel"};
    uint16_t mpduBufferSize{256};
    uint32_t maxAmpduSize{64 * 4 * (700 + 150)};
    uint32_t maxAmpduSize1{64 * 4 * (700 + 150)};
    uint32_t maxAmpduSize2{4 * (700 + 150)}; // payload = 700
    bool grid_search_enable = true;
    uint8_t mode = 2;
    uint32_t nss = 1;
    double simT = 3;
    double txoplimit1 = 0.0;
    double txoplimit2 = 0.0;
    uint32_t use5Gonly = 0;
    uint8_t enable_interference = 0; // 0 disable, 1 enable 2.4G interference, 2 enable 5G interference
    uint32_t seedNumber = 1;

    CommandLine cmd(__FILE__);
    std::filesystem::path filepath = __FILE__;
    cmd.AddValue("link1Pct", "The ratio of packets allocated to link1", link1Pct);
    cmd.AddValue("mcs1", "MCS for 2.4 GHz", mcs1);
    cmd.AddValue("mcs2", "MCS for 5 GHz", mcs2);
    cmd.AddValue("loadrate1", "load rate on 2.4 GHz", r1);
    cmd.AddValue("loadrate2", "load rate on 5 GHz", r2);
    cmd.AddValue("ch1", "channel id on 2.4 GHz", ch1);
    cmd.AddValue("ch2", "channel id on 2.4 GHz", ch2);
    cmd.AddValue("scene_id", "set scene name", sceneId);
    cmd.AddValue("ratectrl", "rate control alg", rateCtrl);
    cmd.AddValue("bawsize", "BA Window Size", mpduBufferSize);
    cmd.AddValue("max_ampdusize", "Max AmpduSize of AP 0", maxAmpduSize);
    cmd.AddValue("max_ampdusize1", "Max AmpduSize of AP 1", maxAmpduSize1);
    cmd.AddValue("max_ampdusize2", "Max AmpduSize of AP 2", maxAmpduSize2);
    cmd.AddValue("gridsearch", "enable gridsearch", grid_search_enable);
    cmd.AddValue("mode", "MLO Mode Setting", mode);
    cmd.AddValue("simt", "simulation time", simT);
    cmd.AddValue("seed", "seed number", seedNumber);
    cmd.AddValue("nss", "mimo", nss);
    cmd.AddValue("txoplimit1", "txop limit for 2.4 GHz", txoplimit1);
    cmd.AddValue("txoplimit2", "txop limit for 5 GHz", txoplimit2);
    cmd.AddValue("5Gonly", "use 5G only", use5Gonly);
    cmd.AddValue("outputfile", "output file", outputfile);
    cmd.AddValue("enable_interference", "enable interference", enable_interference);
    cmd.Parse(argc, argv);

    std::string title = "bw_" + std::to_string(ch1) + "_" + std::to_string(ch2) + "_mcs_" +
                        std::to_string(mcs1) + "_" + std::to_string(mcs2) + "_interference_" +
                        std::to_string(r1) + "_" + std::to_string(r2) + "_txoplimits_" +
                        std::to_string(txoplimit1) + "_" + std::to_string(txoplimit2);
    std::string csv_file = (filepath.parent_path() / ("no_interference_result_" + title + "_thpt.csv")).string();
    std::cout << csv_file << std::endl;

    outputfile = outputfile + "_txoplimits_" + std::to_string(txoplimit1) + 
                "_" + std::to_string(txoplimit2) + "_link1Pct_" + std::to_string(link1Pct) + ".csv";

    // LogComponentEnable("PhyEntity", LOG_LEVEL_DEBUG);
    // bool udp = true;
    uint8_t nLinks = 2;
    RngSeedManager::SetSeed(seedNumber);
    RngSeedManager::SetRun(seedNumber);
    double txPower = 16;
    bool useRts{false};

    int gi = 800;
    Time simulationTime{Seconds(simT)};
    std::string dlAckSeqType{"NO-OFDMA"};
    size_t nStaMlds{1};
    std::vector<size_t> nStaSlds{1, 1};
    std::vector<uint32_t> Cwmin{1, 1};
    std::vector<uint32_t> Cwmax{3, 3};
    uint32_t payloadSize =
        700; // must fit in the max TX duration when transmitting at MCS 0 over an RU of 26 tones
    Time accessReqInterval{0};
    uint32_t maxGroupSize = 1;
    if (useRts)
    {
        Config::SetDefault("ns3::WifiRemoteStationManager::RtsCtsThreshold", StringValue("0"));
        Config::SetDefault("ns3::WifiDefaultProtectionManager::EnableMuRts", BooleanValue(true));
    }

    // Config::SetDefault("ns3::WifiMacQueue::MaxDelay", TimeValue(simulationTime * 2));

    // Disable fragmentation
    Config::SetDefault("ns3::WifiRemoteStationManager::FragmentationThreshold",
                       UintegerValue(std::numeric_limits<uint32_t>::max()));

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
    std::vector<uint32_t> mcs{mcs1, mcs2};
    std::vector<uint32_t> bandwidth{bw1, bw2};
    std::vector<uint32_t> channelnum{ch1, ch2};
    std::string dataModeStr;
    uint64_t nonHtRefRateMbps;
    std::string ctrlRateStr;

    if (rateCtrl == "constant")
    {
        for (uint8_t i = 0; i < nLinks; ++i)
        {
            if (i == 0)
            {
                dataModeStr = "EhtMcs" + std::to_string(mcs[i]);
                nonHtRefRateMbps = EhtPhy::GetNonHtReferenceRate(mcs[i]) / 1e6;
                ctrlRateStr = "ErpOfdmRate" + std::to_string(nonHtRefRateMbps) + "Mbps";
                std::cout << "Link " << std::to_string(i) << " ControlRate: " << ctrlRateStr
                          << " DataMode: " << dataModeStr << std::endl;
                wifi.SetRemoteStationManager(i,
                                             "ns3::ConstantRateWifiManager",
                                             "DataMode",
                                             StringValue(dataModeStr),
                                             "ControlMode",
                                             StringValue(ctrlRateStr));
                dataModeStr = "HeMcs" + std::to_string(mcs[i]);
                nonHtRefRateMbps = HePhy::GetNonHtReferenceRate(mcs[i]) / 1e6;
                ctrlRateStr = "ErpOfdmRate" + std::to_string(nonHtRefRateMbps) + "Mbps";
                std::cout << "2.4G OBSS: ControlRate: " << ctrlRateStr
                          << " DataMode: " << dataModeStr << std::endl;
                wifi2.SetRemoteStationManager("ns3::ConstantRateWifiManager",
                                              "DataMode",
                                              StringValue(dataModeStr),
                                              "ControlMode",
                                              StringValue(ctrlRateStr));
            }
            else
            {
                dataModeStr = "EhtMcs" + std::to_string(mcs[i]);
                nonHtRefRateMbps = EhtPhy::GetNonHtReferenceRate(mcs[i]) / 1e6;
                ctrlRateStr = "OfdmRate" + std::to_string(nonHtRefRateMbps) + "Mbps";
                std::cout << "Link " << std::to_string(i) << " ControlRate: " << ctrlRateStr
                          << " DataMode: " << dataModeStr << std::endl;
                wifi.SetRemoteStationManager(i,
                                             "ns3::ConstantRateWifiManager",
                                             "DataMode",
                                             StringValue(dataModeStr),
                                             "ControlMode",
                                             StringValue(ctrlRateStr));

                mcs[i] = mcs[i] > 11 ? 11 : mcs[i];
                dataModeStr = "HeMcs" + std::to_string(mcs[i]);
                nonHtRefRateMbps = HePhy::GetNonHtReferenceRate(mcs[i]) / 1e6;
                ctrlRateStr = "OfdmRate" + std::to_string(nonHtRefRateMbps) + "Mbps";
                std::cout << "5G OBSS: ControlRate: " << ctrlRateStr << " DataMode: " << dataModeStr
                          << std::endl;
                wifi5.SetRemoteStationManager("ns3::ConstantRateWifiManager",
                                              "DataMode",
                                              StringValue(dataModeStr),
                                              "ControlMode",
                                              StringValue(ctrlRateStr));
            }
        }
    }
    else if (rateCtrl == "minstrel")
    {
        for (uint8_t i = 0; i < nLinks; ++i)
        {
            wifi.SetRemoteStationManager(i, "ns3::MinstrelHtWifiManager");
            if (i == 0)
            {
                wifi2.SetRemoteStationManager("ns3::MinstrelHtWifiManager");
            }
            else
            {
                wifi5.SetRemoteStationManager("ns3::MinstrelHtWifiManager");
            }
        }
    }

    // PD SR
    // wifi.SetObssPdAlgorithm("ns3::ConstantObssPdAlgorithm", "ObssPdLevel", DoubleValue(-72.0));

    /* PropagationLossModel Configuration */
    Ptr<MultiModelSpectrumChannel> spectrumChannel_2 = CreateObject<MultiModelSpectrumChannel>();
    Ptr<LogDistancePropagationLossModel> lossModel_2 =
        CreateObject<LogDistancePropagationLossModel>();
    lossModel_2->SetAttribute("Exponent", DoubleValue(2.0));
    lossModel_2->SetAttribute("ReferenceDistance", DoubleValue(1.0));
    lossModel_2->SetAttribute("ReferenceLoss", DoubleValue(40.046));
    spectrumChannel_2->AddPropagationLossModel(lossModel_2);

    Ptr<MultiModelSpectrumChannel> spectrumChannel_5 = CreateObject<MultiModelSpectrumChannel>();
    Ptr<LogDistancePropagationLossModel> lossModel_5 =
        CreateObject<LogDistancePropagationLossModel>();
    lossModel_5->SetAttribute("Exponent", DoubleValue(3.5));
    lossModel_5->SetAttribute("ReferenceDistance", DoubleValue(1.0));
    lossModel_5->SetAttribute("ReferenceLoss", DoubleValue(50));
    spectrumChannel_5->AddPropagationLossModel(lossModel_5);

    /* PHY Configuration */
    SpectrumWifiPhyHelper phy(nLinks);
    phy.SetPcapDataLinkType(WifiPhyHelper::DLT_IEEE802_11_RADIO);
    if (nss > 1)
    {
        phy.Set("Antennas", UintegerValue(nss));
        phy.Set("MaxSupportedTxSpatialStreams", UintegerValue(nss));
        phy.Set("MaxSupportedRxSpatialStreams", UintegerValue(nss));
    }
    phy.SetErrorRateModel("ns3::TableBasedErrorRateModel");
    phy.AddChannel(spectrumChannel_2, WIFI_SPECTRUM_2_4_GHZ);
    phy.AddChannel(spectrumChannel_5, WIFI_SPECTRUM_5_GHZ);

    for (uint8_t i = 0; i < nLinks; ++i)
    {
        if (i == 0)
            phy.AddPhyToFreqRangeMapping(i, WIFI_SPECTRUM_2_4_GHZ);
        else
            phy.AddPhyToFreqRangeMapping(i, WIFI_SPECTRUM_5_GHZ);
    }
    phy.Set("TxPowerStart", DoubleValue(txPower));
    phy.Set("TxPowerEnd", DoubleValue(txPower));
    phy.EnablePcap("mlo-obss-dl-ucp", apNodes, true);
    for (uint8_t i = 0; i < nLinks; ++i)
    {
        std::string channelSetting =
            "{" + std::to_string(channelnum[i]) + ", " + std::to_string(bandwidth[i]);
        if (i == 0)
            phy.Set(i, "ChannelSettings", StringValue(channelSetting + ", BAND_2_4GHZ, 0}"));
        else
            phy.Set(i, "ChannelSettings", StringValue(channelSetting + ", BAND_5GHZ, 0}"));
    }

    // SLD PHY
    SpectrumWifiPhyHelper phySld2(1);
    phySld2.SetPcapDataLinkType(WifiPhyHelper::DLT_IEEE802_11_RADIO);
    phySld2.SetErrorRateModel("ns3::TableBasedErrorRateModel");
    phySld2.Set("TxPowerStart", DoubleValue(txPower));
    phySld2.Set("TxPowerEnd", DoubleValue(txPower));
    phySld2.AddChannel(spectrumChannel_2, WIFI_SPECTRUM_2_4_GHZ);
    phySld2.Set("ChannelSettings",
                StringValue("{0, " + std::to_string(bandwidth[0]) + ", BAND_2_4GHZ, 0}"));

    SpectrumWifiPhyHelper phySld5(1);
    phySld5.SetPcapDataLinkType(WifiPhyHelper::DLT_IEEE802_11_RADIO);
    phySld5.SetErrorRateModel("ns3::TableBasedErrorRateModel");
    phySld5.Set("TxPowerStart", DoubleValue(txPower));

    phySld5.Set("TxPowerEnd", DoubleValue(txPower));
    phySld5.AddChannel(spectrumChannel_5, WIFI_SPECTRUM_5_GHZ);
    phySld5.Set("ChannelSettings",
                StringValue("{0, " + std::to_string(bandwidth[1]) + ", BAND_5GHZ, 0}"));

    /* MAC Configuration */
    WifiMacHelper mac;
    Ssid bssSsid = Ssid("AP-MLD");
    Ssid obssSsid2 = Ssid("AP-2G");
    Ssid obssSsid5 = Ssid("AP-5G");
    // 1. STA MAC 设置
    mac.SetType("ns3::StaWifiMac",
                "Ssid",
                SsidValue(bssSsid),
                "ActiveProbing",
                BooleanValue(false));
    mldDev = wifi.Install(phy, mac, mldNodes);

    if (enable_interference & 0x01)
    {
        mac.SetType("ns3::StaWifiMac",
                    "Ssid",
                    SsidValue(obssSsid2),
                    "ActiveProbing",
                    BooleanValue(true));
        sldDev2 = wifi2.Install(phySld2, mac, sldNodes2);
    }
    if (enable_interference & 0x02)
    {
        mac.SetType("ns3::StaWifiMac",
                    "Ssid",
                    SsidValue(obssSsid5),
                    "ActiveProbing",
                    BooleanValue(false));
        sldDev5 = wifi5.Install(phySld5, mac, sldNodes5);
    }

    // print mac address of mld
    Ptr<WifiMac> mac1 = DynamicCast<WifiNetDevice>(mldDev.Get(0))->GetMac();
    std::cout << "MLD MAC: " << mldDev.Get(0)->GetAddress() << std::endl;
    for (uint8_t i = 0; i < nLinks; ++i)
    {
        auto fem = mac1->GetFrameExchangeManager(i);
        std::cout << "\t mldDevice " << "linkId " << std::to_string(i)
                  << " mac address: " << fem->GetAddress() << std::endl;
    }
    // print mac address of slds
    if (enable_interference & 0x01)
    {
        std::cout << "SLD2 MAC: " << sldDev2.Get(0)->GetAddress() << std::endl;
    }
    if (enable_interference & 0x02)
    {
        std::cout << "SLD5 MAC: " << sldDev5.Get(0)->GetAddress() << std::endl;
    }

    // 2. AP MAC 设置
    uint64_t beaconInterval = 1024 * 100;
    mac.SetType("ns3::ApWifiMac",
                "BeaconInterval",
                TimeValue(MicroSeconds(beaconInterval)),
                "EnableBeaconJitter",
                BooleanValue(false),
                "Ssid",
                SsidValue(bssSsid));
    apDev = wifi.Install(phy, mac, apNodes.Get(0));
    DynamicCast<WifiNetDevice>(apDev.Get(0))
        ->GetPhy(0)
        ->TraceConnectWithoutContext("PpduTxDuration", MakeCallback(&NotifyPpduTxDurationmld));
    DynamicCast<WifiNetDevice>(apDev.Get(0))
        ->GetPhy(1)
        ->TraceConnectWithoutContext("PpduTxDuration", MakeCallback(&NotifyPpduTxDurationmld));
    if (enable_interference & 0x01)
    {
        mac.SetType("ns3::ApWifiMac",
                    "BeaconInterval",
                    TimeValue(MicroSeconds(beaconInterval)),
                    "EnableBeaconJitter",
                    BooleanValue(false),
                    "Ssid",
                    SsidValue(obssSsid2));
        apDev.Add(wifi2.Install(phySld2, mac, apNodes.Get(1)));
        DynamicCast<WifiNetDevice>(apDev.Get(1))
            ->GetPhy(0)
            ->TraceConnectWithoutContext("PpduTxDuration", MakeCallback(&NotifyPpduTxDuration));
    }
    if (enable_interference & 0x02)
    {
        mac.SetType("ns3::ApWifiMac",
                    "BeaconInterval",
                    TimeValue(MicroSeconds(beaconInterval)),
                    "EnableBeaconJitter",
                    BooleanValue(false),
                    "Ssid",
                    SsidValue(obssSsid5));
        apDev.Add(wifi5.Install(phySld5, mac, apNodes.Get(2)));
    }

    // print mac address of ap
    std::cout << "AP0 MAC: " << apDev.Get(0)->GetAddress() << std::endl;
    Ptr<WifiMac> mac2 = DynamicCast<WifiNetDevice>(apDev.Get(0))->GetMac();
    for (uint8_t i = 0; i < nLinks; ++i)
    {
        auto fem = mac2->GetFrameExchangeManager(i);
        std::cout << "\t apDevice " << "linkId " << std::to_string(i)
                  << " mac address: " << fem->GetAddress() << std::endl;
    }
    if (enable_interference & 0x01)
    {
        std::cout << "AP1 MAC: " << apDev.Get(1)->GetAddress() << std::endl;
    }
    if (enable_interference & 0x02)
    {
        std::cout << "AP2 MAC: " << apDev.Get(2)->GetAddress() << std::endl;
    }
    NetDeviceContainer devices;
    devices.Add(apDev);
    devices.Add(mldDev);
    devices.Add(sldDev2);
    devices.Add(sldDev5);
    wifi.AssignStreams(devices, seedNumber);

    // Set guard interval, MPDU buffer size, MaxAmsduSize, MaxAmpduSize
    Config::Set("/NodeList/*/DeviceList/*/$ns3::WifiNetDevice/HeConfiguration/GuardInterval",
                TimeValue(NanoSeconds(gi)));
    Config::Set("/NodeList/0/DeviceList/*/$ns3::WifiNetDevice/Mac/BE_MaxAmsduSize",
                UintegerValue(0));

    if (enable_interference & 0x01)
    {
        Config::Set("/NodeList/1/DeviceList/*/$ns3::WifiNetDevice/Mac/BE_MaxAmpduSize",
                    UintegerValue(maxAmpduSize1));
    }
    if (enable_interference & 0x02)
    {
        Config::Set("/NodeList/2/DeviceList/*/$ns3::WifiNetDevice/Mac/BE_MaxAmpduSize",
                    UintegerValue(maxAmpduSize2));
    }

    Config::Set("/NodeList/0/DeviceList/*/$ns3::WifiNetDevice/Mac/BE_MaxAmpduSize",
                UintegerValue(maxAmpduSize));
    Config::Set("/NodeList/*/DeviceList/*/$ns3::WifiNetDevice/Mac/MpduBufferSize",
                UintegerValue(mpduBufferSize));

    /* Mobility Configuration */
    MobilityHelper mobility;
    Ptr<ListPositionAllocator> positionAlloc = CreateObject<ListPositionAllocator>();
    mobility.SetMobilityModel("ns3::ConstantPositionMobilityModel");
    positionAlloc->Add(Vector(0.0, 0.0, 0.0));
    positionAlloc->Add(Vector(5.0, 0.0, 0.0));
    positionAlloc->Add(Vector(0.0, 5.0, 0.0));
    positionAlloc->Add(Vector(0.0, 0.0, 1.0));
    positionAlloc->Add(Vector(5.0, 0.0, 1.0));
    positionAlloc->Add(Vector(0.0, 5.0, 1.0));
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
    /* Setting applications */
    ApplicationContainer dlserverApp;
    ApplicationContainer dlserverAppObss2;
    ApplicationContainer dlserverAppObss5;
    // 1. DL UDL configure
    // DL udp flow
    uint16_t port = 9;
    // Address localAddress(InetSocketAddress(Ipv4Address::GetAny(), port));
    UdpServerHelper server(port);
    dlserverApp = server.Install(mldNodes);
    seedNumber += server.AssignStreams(mldNodes, seedNumber);
    dlserverApp.Start(Seconds(0.0));
    dlserverApp.Stop(simulationTime + Seconds(5.0));
    // AP 0
    const auto maxLoad2 = EhtPhy::GetDataRate(mcs[0], bandwidth[0], NanoSeconds(gi), 1);
    const auto maxLoad5 = EhtPhy::GetDataRate(mcs[1], bandwidth[1], NanoSeconds(gi), 1);
    const auto maxLoad = (maxLoad2 + maxLoad5);
    std::cout << "maxload = " << std::to_string((maxLoad2 + maxLoad5) / 1e6)
              << "Mbps; 2.4 GHz: " << std::to_string(maxLoad2 / 1e6)
              << "Mbps, 5 GHz: " << std::to_string(maxLoad5 / 1e6) << "Mbps" << std::endl;
    ApplicationContainer clientApp1;
    if (enable_interference & 0x01)
    {
        address.SetBase("10.0.1.0", "255.255.255.0");
        Ipv4InterfaceContainer apNodeInterface2 = address.Assign(apDev.Get(1));
        Ipv4InterfaceContainer sldNodeInterface2 = address.Assign(sldDev2);
        std::cout << "AP1 IP: " << apNodeInterface2.GetAddress(0) << std::endl;
        std::cout << "  Link 0 SLD-1: " << sldNodeInterface2.GetAddress(0) << std::endl;
        dlserverAppObss2 = server.Install(sldNodes2);
        seedNumber += server.AssignStreams(sldNodes2, seedNumber);
        dlserverAppObss2.Start(Seconds(0.2));
        dlserverAppObss2.Stop(simulationTime + Seconds(5.0));
        // AP 1
        auto packetInterval2 = payloadSize * 8.0 / maxLoad2 / r1;
        UdpClientHelper client1(sldNodeInterface2.GetAddress(0), port);
        client1.SetAttribute("MaxPackets", UintegerValue(0));
        client1.SetAttribute("Interval", TimeValue(Seconds(packetInterval2)));
        client1.SetAttribute("PacketSize", UintegerValue(payloadSize));
        clientApp1.Add(client1.Install(apNodes.Get(1)));
        seedNumber += client1.AssignStreams(apNodes.Get(1), seedNumber);
        clientApp1.Start(Seconds(1));
        clientApp1.Stop(simulationTime + Seconds(5.0));
    }
    if (enable_interference & 0x02)
    {
        address.SetBase("10.0.2.0", "255.255.255.0");
        Ipv4InterfaceContainer apNodeInterface5 = address.Assign(apDev.Get(2));
        Ipv4InterfaceContainer sldNodeInterface5 = address.Assign(sldDev5);
        std::cout << "AP2 IP: " << apNodeInterface5.GetAddress(0) << std::endl;
        std::cout << "  Link 1 SLD-1: " << sldNodeInterface5.GetAddress(0) << std::endl;
        dlserverAppObss5 = server.Install(sldNodes5);
        seedNumber += server.AssignStreams(sldNodes5, seedNumber);
        dlserverAppObss5.Start(Seconds(0.4));
        dlserverAppObss5.Stop(simulationTime + Seconds(1.0));
        // AP 2
        // auto packetInterval5 = payloadSize * 8.0 / maxLoad5 / r2;
        UdpClientHelper client2(sldNodeInterface5.GetAddress(0), port);
        client2.SetAttribute("MaxPackets", UintegerValue(1));
        client2.SetAttribute("Interval", TimeValue(Seconds(100))); // 无干扰？
        client2.SetAttribute("PacketSize", UintegerValue(payloadSize));
        ApplicationContainer clientApp2 = client2.Install(apNodes.Get(2));
        seedNumber += client2.AssignStreams(apNodes.Get(2), seedNumber);
        clientApp2.Start(Seconds(1.0));
        clientApp2.Stop(simulationTime + Seconds(5.0));
    }
    // print Ip address of devices
    std::cout << "AP0 IP: " << apNodeInterface.GetAddress(0) << std::endl;
    std::cout << "STA IP: " << std::endl;
    for (size_t i = 0; i < nStaMlds; ++i)
    {
        std::cout << "  MLD-" << std::to_string(i) + ": " << mldNodeInterface.GetAddress(i)
                  << std::endl;
    }

    const auto packetInterval = payloadSize * 8.0 / maxLoad ;
    UdpClientHelper client(mldNodeInterface.GetAddress(0), port);
    client.SetAttribute("MaxPackets", UintegerValue(0));
    client.SetAttribute("Interval",
                        TimeValue(Seconds(packetInterval / 30))); // 在0.01的分配比例下保证饱和
    client.SetAttribute("PacketSize", UintegerValue(payloadSize));
    ApplicationContainer clientApp = client.Install(apNodes.Get(0));
    seedNumber += client.AssignStreams(apNodes.Get(0), seedNumber);
    clientApp.Start(Seconds(1.0));
    clientApp.Stop(simulationTime + Seconds(1.0));

    // Enable TID-to-Link Mapping for AP and MLD STAs
    for (auto i = mldDev.Begin(); i != mldDev.End(); ++i)
    {
        auto wifiDev = DynamicCast<WifiNetDevice>(*i);
        wifiDev->GetMac()->GetEhtConfiguration()->SetAttribute(
            "TidToLinkMappingNegSupport",
            EnumValue(WifiTidToLinkMappingNegSupport::ANY_LINK_SET));
    }

    std::string mldMappingStr = "0,1,2,3,4,5,6,7 0,1";

    if (use5Gonly == 1)
    {
        mldMappingStr = "0,1,2,3,4,5,6,7 1";
    }
    else if (use5Gonly == 2)
    {
        mldMappingStr = "0,1,2,3,4,5,6,7 0";
    }
    for (auto i = mldDev.Begin(); i != mldDev.End(); ++i)
    {
        auto wifiDev = DynamicCast<WifiNetDevice>(*i);
        wifiDev->GetMac()->SetAttribute("ActiveProbing", BooleanValue(true));
        wifiDev->GetMac()->GetEhtConfiguration()->SetAttribute("TidToLinkMappingDl",
                                                               StringValue(mldMappingStr));
        wifiDev->GetMac()->GetEhtConfiguration()->SetAttribute("TidToLinkMappingUl",
                                                               StringValue(mldMappingStr));
    }

    auto apWifiDev = DynamicCast<WifiNetDevice>(apDev.Get(0));
    apWifiDev->GetMac()->GetEhtConfiguration()->SetAttribute("TidToLinkMappingDl",
                                                             StringValue(mldMappingStr));
    apWifiDev->GetMac()->GetEhtConfiguration()->SetAttribute("TidToLinkMappingUl",
                                                             StringValue(mldMappingStr));

    Config::Set("/NodeList/0/DeviceList/*/$ns3::WifiNetDevice/Mac/BE_Txop/MaxGroupSize",
                UintegerValue(maxGroupSize));
    Config::Set("/NodeList/0/DeviceList/*/$ns3::WifiNetDevice/Mac/BE_Txop/Period",
                TimeValue(Seconds(simT))); 
    Config::Set("/NodeList/0/DeviceList/*/$ns3::WifiNetDevice/Mac/BE_Txop/Mode",
                UintegerValue(mode)); 
    Config::Set("/NodeList/3/DeviceList/*/$ns3::WifiNetDevice/Mac/BE_Txop/Mode", UintegerValue(0x01 << 2)); // 只负责接收，无msdu_grouper
    if (mode == 2)
    {
        link1Pct = std::round(link1Pct * 1000) / 1000;
        Config::Set("/NodeList/0/DeviceList/*/$ns3::WifiNetDevice/Mac/BE_Txop/Link1Pct",
                    DoubleValue(link1Pct));
    }
    Config::Set("/NodeList/0/DeviceList/*/$ns3::WifiNetDevice/Mac/BE_Txop/GridSearchEnable",
                BooleanValue(true)); 

    /* BSS EDCA */
    Config::Set("/NodeList/0/DeviceList/*/$ns3::WifiNetDevice/Mac/BE_Txop/Aifsns",
                AttributeContainerValue<UintegerValue>(std::list<uint64_t>{3, 3}));
    Config::Set("/NodeList/0/DeviceList/*/$ns3::WifiNetDevice/Mac/BE_Txop/MinCws",
                AttributeContainerValue<UintegerValue>(std::vector<uint32_t>{Cwmin[0], Cwmin[1]}));
    Config::Set("/NodeList/0/DeviceList/*/$ns3::WifiNetDevice/Mac/BE_Txop/MaxCws",
                AttributeContainerValue<UintegerValue>(std::vector<uint32_t>{Cwmax[0], Cwmax[1]}));
    std::list<Time> txopLimitList = {MicroSeconds(txoplimit1) * 32, MicroSeconds(txoplimit2) * 32};
    Config::Set("/NodeList/0/DeviceList/*/$ns3::WifiNetDevice/Mac/BE_Txop/TxopLimits",
                AttributeContainerValue<TimeValue>(txopLimitList));

    /* OBSS EDCA */
    Config::Set("/NodeList/1/DeviceList/*/$ns3::WifiNetDevice/Mac/BE_Txop/Aifsns",
                AttributeContainerValue<UintegerValue>(std::list<uint64_t>{3}));
    Config::Set("/NodeList/1/DeviceList/*/$ns3::WifiNetDevice/Mac/BE_Txop/MinCws",
                AttributeContainerValue<UintegerValue>(std::vector<uint32_t>{1}));
    Config::Set("/NodeList/1/DeviceList/*/$ns3::WifiNetDevice/Mac/BE_Txop/MaxCws",
                AttributeContainerValue<UintegerValue>(std::vector<uint32_t>{3}));
    Config::Set("/NodeList/2/DeviceList/*/$ns3::WifiNetDevice/Mac/BE_Txop/Aifsns",
                AttributeContainerValue<UintegerValue>(std::list<uint64_t>{3}));
    Config::Set("/NodeList/2/DeviceList/*/$ns3::WifiNetDevice/Mac/BE_Txop/MinCws",
                AttributeContainerValue<UintegerValue>(std::vector<uint32_t>{1}));
    Config::Set("/NodeList/2/DeviceList/*/$ns3::WifiNetDevice/Mac/BE_Txop/MaxCws",
                AttributeContainerValue<UintegerValue>(std::vector<uint32_t>{3}));

    // phy.EnablePcap("ap0-trace", apDev.Get(0));
    // phySld2.EnablePcap("ap1-trace", apDev.Get(1));
    // phySld5.EnablePcap("ap2-trace", apDev.Get(2));
    // phy.EnablePcap("mld-trace", mldDev.Get(0));
    // phySld2.EnablePcap("sld2-trace", sldDev2.Get(0));
    // phySld5.EnablePcap("sld5-trace", sldDev5.Get(0));
    Config::ConnectWithoutContext(
        "/NodeList/0/DeviceList/*/$ns3::WifiNetDevice/Mac/BE_Txop/GetNextParams",
        MakeCallback(&SaveParams));
    Config::ConnectWithoutContext(
        "/NodeList/0/DeviceList/*/$ns3::WifiNetDevice/Mac/BE_Txop/GetTxopTimeStats",
        MakeCallback(&SavetxopTime));
    std::fstream file;
    file.open(outputfile, std::ios::out);
    file << "type" << ",PPDU Start,PPDU End,AMPDU Size" << std::endl;
    file.close();
    file.open("txop" + outputfile, std::ios::out);
    file << "link" << ",Txop Start,Txop End" << std::endl;
    file.close();
    Simulator::Stop(simulationTime + Seconds(1.0) + MilliSeconds(1));
    Simulator::Run();

    std::vector<uint64_t> dlCumulRxBytes(1, 0);

    dlCumulRxBytes = GetRxBytes(true, dlserverApp, payloadSize);
    auto dlrxBytes = std::accumulate(dlCumulRxBytes.cbegin(), dlCumulRxBytes.cend(), 0.0);
    dlthroughput = (dlrxBytes * 8) / simulationTime.GetMicroSeconds(); // Mbit/s
    std::cout << "DL Throughput: \t\t\t" << dlthroughput << " Mbit/s" << std::endl;

    Simulator::Destroy();

    std::ofstream fout(csv_file, std::ios::app); // 使用追加模式

    // 检查文件是否为空，如果为空则写入表头
    if (fout.tellp() == 0) { 
        fout << "No, Mode, CWmin1, CWmax1, CWmin2, CWmax2, Aifsn1, Aifsn2, TxopLimit1, TxopLimit2, "
                "RTS_CTS1, RTS_CTS2, MaxSlrc1, "
                "MaxSsrc1, MaxSlrc2, MaxSsrc2, RedundancyThreshold1, RedundancyThreshold2, "
                "RedundancyFixedNumber1, "
                "RedundancyFixedNumber2, link1Pct, BlockCnt1, BlockCnt2, BlockCnt1_True, BlockCnt2_True, "
                "TxopTime1(us), TxopTime2(us), TxopCnt1, TxopCnt2, MaxAmpduLength1, MaxAmpduLength2, "
                "p1, p2, Occupancy Rate 1, Occupancy Rate 2, datarate1, datarate2, Thp1(Mbps), Thp2(Mbps), Throughput(Mbps)"
             << std::endl;
    }
    if (res){
        auto params = res->params;
        fout << params["No"][0] << ", " << (uint32_t)mode << ", " << params["CWmins"][0] << ", "
             << params["CWmaxs"][0] << ", " << params["CWmins"][1] << ", " << params["CWmaxs"][1]
             << ", " << params["Aifsns"][0] << ", " << params["Aifsns"][1] << ", "
             << params["TxopLimits"][0] << ", " << params["TxopLimits"][1] << ", "
             << params["RTS_CTS"][0] << ", " << params["RTS_CTS"][1] << ", "
             << params["MaxSlrcs"][0] << ", " << params["MaxSsrcs"][0] << ", "
             << params["MaxSlrcs"][1] << ", " << params["MaxSsrcs"][1] << ", "
             << params["RedundancyThresholds"][0] << ", " << params["RedundancyThresholds"][1]
             << ", " << params["RedundancyFixedNumbers"][0] << ", "
             << params["RedundancyFixedNumbers"][1] << ", " << res->link1Pct << ","
             << res->blockCnt[0] << ", " << res->blockCnt[1] << ", " << res->blockCnt_tr[0] << ", "
             << res->blockCnt_tr[1] << ", " << res->txopTime[0] << ", " << res->txopTime[1] << ", "
             << res->txopNum[0] << ", " << res->txopNum[1] << ", " << res->maxAmpduLength[0] << ", "
             << res->maxAmpduLength[1] << ", " << res->p[0] << ", " << res->p[1] << ", "
             << res->occ[0] << ", " << res->occ[1] << ", " << res->datarate[0] << ", "
             << res->datarate[1] << ", " << res->thpt[0] << ", " << res->thpt[1] << ", "
             << dlthroughput << std::endl;
    }
    return 0;
}
