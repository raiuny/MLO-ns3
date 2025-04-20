/*
 * Copyright (c) 2024 Huazhong University of Science and Technology
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
 */

#include "wifi-tx-stats-helper.h"

#include <ns3/nstime.h>
#include <ns3/wifi-mpdu.h>
#include <ns3/packet.h>
#include <ns3/wifi-net-device.h>
#include <ns3/net-device-container.h>
#include <ns3/wifi-mac.h>
#include <ns3/qos-txop.h>
#include <ns3/wifi-mac-queue.h>
#include <ns3/wifi-net-device.h>
#include <ns3/wifi-phy.h>
#include <ns3/block-ack-manager.h>
#include <ns3/frame-exchange-manager.h>
#include <ns3/gnuplot-helper.h>
#include <fstream>
#include <cmath>
#include "ns3/log.h"

namespace ns3
{
NS_LOG_COMPONENT_DEFINE("WifiTxStatsHelper");
WifiTxStatsHelper::WifiTxStatsHelper()
{
}

void
WifiTxStatsHelper::Enable(NetDeviceContainer devices)
{
    m_traceSink = CreateObject<WifiTxStatsTraceSink>();
    global_nodes = NodeContainer().GetGlobal();
    for(uint32_t i = 0; i < global_nodes.GetN(); i++)
    {
        if(DynamicCast<WifiNetDevice>(global_nodes.Get(i)->GetDevice(0)))
        {
            Mac48Address addr = DynamicCast<WifiNetDevice>(global_nodes.Get(i)->GetDevice(0))->GetMac()->GetAddress();
            std::cout<<"node "<< i <<" mac address: "<<addr<<std::endl;
            MacAddresstoNodeId[addr]=i;
        }
    }
    m_traceSink->SetMacAddresstoNodeId(MacAddresstoNodeId);

    for (auto dev = devices.Begin(); dev != devices.End(); dev++)
    {
        Ptr<WifiNetDevice> wifiDev = DynamicCast<WifiNetDevice>(*dev);
        for (auto& ac : m_aciToString)
        {
            wifiDev->GetMac()->GetTxopQueue(ac.first)->TraceConnectWithoutContext(
                "Enqueue",
                MakeCallback(&WifiTxStatsTraceSink::NotifyMacEnqueue, m_traceSink));
            wifiDev->GetMac()->GetQosTxop(ac.first)->GetBaManager()->TraceConnectWithoutContext(
                "AckedMpdu",
                MakeCallback(&WifiTxStatsTraceSink::NotifyAcked, m_traceSink));
            wifiDev->GetMac()->GetTxopQueue(ac.first)->TraceConnectWithoutContext(
                "Dequeue",
                MakeCallback(&WifiTxStatsTraceSink::NotifyMacDequeue, m_traceSink));
        }
        
        for (int i = 0; i < wifiDev->GetNPhys(); i++)
        {
            wifiDev->GetMac()->GetFrameExchangeManager(i)->TraceConnectWithoutContext(
                "AckedMpdu",
                MakeCallback(&WifiTxStatsTraceSink::NotifyAcked, m_traceSink));
            wifiDev->GetPhy(i)->TraceConnectWithoutContext(
                "PhyTxBegin",
                MakeCallback(&WifiTxStatsTraceSink::NotifyTxStart, m_traceSink));
            wifiDev->GetPhy(i)->TraceConnectWithoutContext(
                "PhyTxEnd",
                MakeCallback(&WifiTxStatsTraceSink::NotifyTxEnd, m_traceSink));
        }
    }
}

WifiTxFinalStatistics
WifiTxStatsHelper::GetFinalStatistics()
{
    return m_traceSink->DoGetFinalStatistics();
}

const WifiPktTxRecordMap&
WifiTxStatsHelper::GetSuccessInfoMap()
{
    return m_traceSink->DoGetSuccessInfoMap();
}

std::vector<WifiTxPerPktRecord>
WifiTxStatsHelper::GetSuccessPktsPerNode(NodeIdPair nodepair)
{
    std::vector<WifiTxPerPktRecord> results;
    auto it = m_traceSink->DoGetSuccessInfoMap().find(nodepair);
    if (it != m_traceSink->DoGetSuccessInfoMap().end())
    {
        for (const auto& linkMap : it->second)
        {
            results.insert(results.end(),
                        linkMap.second.begin(),
                        linkMap.second.end());
        }
    }
    return results;
}

std::vector<WifiTxPerPktRecord>
WifiTxStatsHelper::GetSuccessPktsTotal()
{
    std::vector<WifiTxPerPktRecord> results;
    for(const auto& nodeMap : m_traceSink->DoGetSuccessInfoMap())
    {
        for (const auto& linkMap : nodeMap.second)
        {
            results.insert(results.end(),
                        linkMap.second.begin(),
                        linkMap.second.end());
        }
    }
    return results;
}

const WifiPktNodeIdMap&
WifiTxStatsHelper::GetFailureInfoMap()
{
    return m_traceSink->DoGetFailureInfoMap();
}

void
WifiTxStatsHelper::Start(Time startTime)
{
    Simulator::Schedule(startTime, &WifiTxStatsTraceSink::DoStart, m_traceSink);
}

void
WifiTxStatsHelper::Stop(Time stopTime)
{
    Simulator::Schedule(stopTime, &WifiTxStatsTraceSink::DoStop, m_traceSink);
}

void
WifiTxStatsHelper::Reset()
{
    m_traceSink->DoReset();
}

NS_OBJECT_ENSURE_REGISTERED(WifiTxStatsTraceSink);

WifiTxStatsTraceSink::WifiTxStatsTraceSink() : m_statsCollecting(false)
{
}

TypeId
WifiTxStatsTraceSink::GetTypeId()
{
    static TypeId tid =
        TypeId("ns3::WifiTxStatsTraceSink")
            .SetParent<Object>()
            .SetGroupName("Wifi")
            .AddConstructor<WifiTxStatsTraceSink>();
    return tid;
}

void
WifiTxStatsTraceSink::DoStart()
{
    m_statsCollecting = true;
}

void
WifiTxStatsTraceSink::DoStop()
{
    m_statsCollecting = false;
}

void
WifiTxStatsTraceSink::DoReset()
{
    m_inflightMap.clear();
    m_successMap.clear();
    m_failureMap.clear();
    m_lastdequeueTimeMap.clear();
}

WifiTxFinalStatistics
WifiTxStatsTraceSink::DoGetFinalStatistics()
{
    WifiTxFinalStatistics results;
    std::map<NodeIdPair, uint64_t> numSuccessPerNodePair;
    std::map<NodeIdPair /* Node ID */, uint64_t> numRetransmissionPerNodePair;
    std::map<NodeIdPair /* Node ID */,
        std::map<uint8_t /* Link ID */, double>> totalE2eDelayPerNodePairLink;
    double totalE2eDelay{0};
    std::vector<WifiTxPerPktRecord> all_pktrecords;

    // Iterate through success map
    for (const auto& nodeMap : m_successMap)
    {
        /*计算每一个包的端到端时延*/ 
        for (const auto& linkMap : nodeMap.second)
        {
            all_pktrecords.insert(all_pktrecords.end(),linkMap.second.begin(),linkMap.second.end());
            for (const auto& record : linkMap.second)
            {   
                results.m_successSizePerNodePairLink[nodeMap.first][linkMap.first] += record.m_packetSize;
                results.m_numSuccessPerNodePairLink[nodeMap.first][linkMap.first]++;
                totalE2eDelayPerNodePairLink[nodeMap.first][linkMap.first] +=
                                record.m_ackTime.ToDouble(Time::MS) - record.m_enqueueTime.ToDouble(Time::MS);
                numSuccessPerNodePair[nodeMap.first]++;
                if (record.m_failures > 0)
                {
                    results.m_numRetransmittedPerNodePair[nodeMap.first]++;
                    numRetransmissionPerNodePair[nodeMap.first] += record.m_failures;
                }
            }

            // 按照端到端时延排序，用于计算95%，99%端到端时延
            auto records_copy = linkMap.second;
            std::sort(records_copy.begin(), records_copy.end(), [](const WifiTxPerPktRecord &a, const WifiTxPerPktRecord &b) {
                return a.m_ackTime-a.m_enqueueTime < b.m_ackTime-b.m_enqueueTime;
            });
            size_t index99 = static_cast<size_t>(std::ceil(records_copy.size() * 0.99)) - 1;
            size_t index95 = static_cast<size_t>(std::ceil(records_copy.size() * 0.95)) - 1;
            results.m_099E2eDelayPerNodePairLink[nodeMap.first][linkMap.first] = 
                        std::accumulate(records_copy.begin(),records_copy.begin()+index99,0.0,
                        [&](double sum,const WifiTxPerPktRecord & record){return sum+record.m_ackTime.ToDouble(Time::MS) - record.m_enqueueTime.ToDouble(Time::MS);})/index99;
            results.m_095E2eDelayPerNodePairLink[nodeMap.first][linkMap.first] = 
                        std::accumulate(records_copy.begin(),records_copy.begin()+index95,0.0,
                        [&](double sum,const WifiTxPerPktRecord & record){return sum+record.m_ackTime.ToDouble(Time::MS) - record.m_enqueueTime.ToDouble(Time::MS);})/index95;
            results.m_meanE2eDelayPerNodePairLink[nodeMap.first][linkMap.first] =
                        totalE2eDelayPerNodePairLink[nodeMap.first][linkMap.first] / results.m_numSuccessPerNodePairLink[nodeMap.first][linkMap.first];

            // 按照接入时延排序，用于计算95%，99%接入时延
            std::sort(records_copy.begin(), records_copy.end(), [](const WifiTxPerPktRecord &a, const WifiTxPerPktRecord &b) {
                return a.m_ackTime-a.m_holtime < b.m_ackTime-b.m_holtime;
            });
            results.m_099AccessDelayPerNodePairLink[nodeMap.first][linkMap.first] = 
                        std::accumulate(records_copy.begin(),records_copy.begin()+index99,0.0,
                        [&](double sum,const WifiTxPerPktRecord & record){return sum+record.m_ackTime.ToDouble(Time::MS) - record.m_holtime.ToDouble(Time::MS);})/index99;
            results.m_095AccessDelayPerNodePairLink[nodeMap.first][linkMap.first] = 
                        std::accumulate(records_copy.begin(),records_copy.begin()+index95,0.0,
                        [&](double sum,const WifiTxPerPktRecord & record){return sum+record.m_ackTime.ToDouble(Time::MS) - record.m_holtime.ToDouble(Time::MS);})/index95;
            results.m_meanAccessDelayPerNodePairLink[nodeMap.first][linkMap.first] =
                                                std::accumulate(records_copy.begin(),records_copy.end(),0.0,
                        [&](double sum,const WifiTxPerPktRecord & record){return sum+record.m_ackTime.ToDouble(Time::MS) - record.m_holtime.ToDouble(Time::MS);})/records_copy.size();

            // 按照排队时延排序，用于计算95%，99%排队时延
            std::sort(records_copy.begin(), records_copy.end(), [](const WifiTxPerPktRecord &a, const WifiTxPerPktRecord &b) {
                return a.m_holtime-a.m_enqueueTime < b.m_holtime-b.m_enqueueTime;
            });
            results.m_099QueueDelayPerNodePairLink[nodeMap.first][linkMap.first] = 
                        std::accumulate(records_copy.begin(),records_copy.begin()+index99,0.0,
                        [&](double sum,const WifiTxPerPktRecord & record){return sum+record.m_holtime.ToDouble(Time::MS) - record.m_enqueueTime.ToDouble(Time::MS);})/index99;
            results.m_095QueueDelayPerNodePairLink[nodeMap.first][linkMap.first] = 
                        std::accumulate(records_copy.begin(),records_copy.begin()+index95,0.0,
                        [&](double sum,const WifiTxPerPktRecord & record){return sum+record.m_holtime.ToDouble(Time::MS) - record.m_enqueueTime.ToDouble(Time::MS);})/index95;
            results.m_meanQueueDelayPerNodePairLink[nodeMap.first][linkMap.first] =
                                                std::accumulate(records_copy.begin(),records_copy.end(),0.0,
                        [&](double sum,const WifiTxPerPktRecord & record){return sum+record.m_holtime.ToDouble(Time::MS) - record.m_enqueueTime.ToDouble(Time::MS);})/records_copy.size();
        }
    }

    // 计算每个nodepair的最终失败包数
    for (const auto& nodeMap : m_failureMap)
    {
        results.m_numFinalFailedPerNodePair[nodeMap.first] += nodeMap.second.size();
    }
    // 计算所有的最终失败包数
    for (const auto& nodeMap : results.m_numFinalFailedPerNodePair)
    {
        results.m_numFinalFailedTotal += nodeMap.second;
    }
    // 计算所有的成功包数
    for (const auto& nodeMap : results.m_numSuccessPerNodePairLink)
    {
        for (const auto& linkMap : nodeMap.second)
        {
            results.m_numSuccessTotal += linkMap.second;
        }
    }
    // 计算所有的平均端到端时延
    for (const auto& nodeMap : totalE2eDelayPerNodePairLink)
    {
        for (const auto& linkMap : nodeMap.second)
        {
            totalE2eDelay += linkMap.second;
        }
    }
    results.m_meanE2eDelayTotal = totalE2eDelay / results.m_numSuccessTotal;
    // 计算所有的重传包数目
    for (const auto& nodeMap : results.m_numRetransmittedPerNodePair)
    {
        results.m_numRetransmittedTotal += nodeMap.second;
    }
    // 计算每个nodepair的平均失败率
    uint64_t numRetransmissionTotal = 0;
    for (const auto& nodeMap : numRetransmissionPerNodePair)
    {
        results.m_avgFailuresPerNodePair[nodeMap.first] = (long double)nodeMap.second / numSuccessPerNodePair[nodeMap.first];
        numRetransmissionTotal += nodeMap.second;
    }
    //计算总的平均失败率
    results.m_avgFailuresTotal = (long double)numRetransmissionTotal / results.m_numSuccessTotal;

    return results;
}

const WifiPktTxRecordMap&
WifiTxStatsTraceSink::DoGetSuccessInfoMap()
{
    return m_successMap;
}

const WifiPktNodeIdMap&
WifiTxStatsTraceSink::DoGetFailureInfoMap()
{
    return m_failureMap;
}

void
WifiTxStatsTraceSink::NotifyMacEnqueue(Ptr<const WifiMpdu> mpdu)
{
    if (mpdu->GetHeader().IsData())
    {   
        // if (mpdu->GetPacket()->GetAdjustment() == mpdu->GetPacketSize()) std::cout << mpdu->GetPacketSize() << std::endl;
        if (mpdu->GetPacketSize() == 0 || mpdu->GetPacketSize() == 36 || mpdu->GetPacket()->GetAdjustment() == mpdu->GetPacketSize())
        {
            // exclude Null frame
            return;
        }
        WifiTxPerPktRecord record;
        // if (mpdu->GetPacketSize() != 596)
            // std::cout << mpdu->GetPacketSize() << std::endl;
        record.m_dstNodeId = MacAddresstoNodeId[mpdu->GetDestinationAddress()];
        record.m_srcNodeId = Simulator::GetContext();
        NS_LOG_DEBUG("Enqueue packet: " << mpdu->GetPacket()->GetUid()<<", from: "<<record.m_srcNodeId<<", to: "<<record.m_dstNodeId);
        record.m_enqueueTime = Simulator::Now();
        record.m_tid = mpdu->GetHeader().GetQosTid();
        m_inflightMap[mpdu->GetPacket()->GetUid()] = record;
    }
}

void
WifiTxStatsTraceSink::NotifyTxStart(Ptr<const Packet> pkt, double txPowerW)
{  
    auto mapIt = m_inflightMap.find(pkt->GetUid());
    if (mapIt != m_inflightMap.end())
    {
        NS_LOG_DEBUG("TxStart packet: " << pkt->GetUid()<<", from: "<<mapIt->second.m_srcNodeId<<", to: "<<mapIt->second.m_dstNodeId);
        if (!mapIt->second.m_txStarted)
        {
            mapIt->second.m_txStarted = true;
            mapIt->second.m_txStartTime = Simulator::Now();
        }
        else
        {
            mapIt->second.m_failures++;
        }
    }
}

void
WifiTxStatsTraceSink::NotifyTxEnd(Ptr<const Packet> pkt)
{
    auto mapIt = m_inflightMap.find(pkt->GetUid());
    if (mapIt != m_inflightMap.end())
    {
        NS_LOG_DEBUG("TxEnd packet: " << pkt->GetUid()<<", from: "<<mapIt->second.m_srcNodeId<<", to: "<<mapIt->second.m_dstNodeId);
        mapIt->second.m_txEndTime = Simulator::Now();        
    }
}

void
WifiTxStatsTraceSink::NotifyAcked(Ptr<const WifiMpdu> mpdu,uint8_t linkId)
{
    auto mapIt = m_inflightMap.find(mpdu->GetPacket()->GetUid());
    if (mapIt != m_inflightMap.end())
    {
        NS_LOG_DEBUG("Acked packet: " << mpdu->GetPacket()->GetUid()<<", from: "<<mapIt->second.m_srcNodeId<<", to: "<<mapIt->second.m_dstNodeId);
        mapIt->second.m_acked = true;
        mapIt->second.m_ackTime = Simulator::Now();
        mapIt->second.m_successLinkId = linkId;
        // std::cout << "link at " << (uint32_t)linkId << std::endl; 
        mapIt->second.m_packetSize = mpdu->GetPacketSize();
    }
}

void
WifiTxStatsTraceSink::NotifyMacDequeue(Ptr<const WifiMpdu> mpdu)
{
    auto mapIt = m_inflightMap.find(mpdu->GetPacket()->GetUid());
    if (mapIt != m_inflightMap.end())
    {
        mapIt->second.m_dequeued = true;
        mapIt->second.m_dequeueTime = Simulator::Now();
        mapIt->second.m_seqNum = mpdu->GetHeader().GetSequenceNumber();
        mapIt->second.m_holtime = mapIt->second.m_enqueueTime;
        // if(m_lastdequeueTimeMap.find(mapIt->second.m_srcNodeId)==m_lastdequeueTimeMap.end()
        // ||m_lastdequeueTimeMap[mapIt->second.m_srcNodeId].find(mapIt->second.m_successLinkId) == m_lastdequeueTimeMap[mapIt->second.m_srcNodeId].end())
        // {
        //     m_lastdequeueTimeMap[mapIt->second.m_srcNodeId][mapIt->second.m_successLinkId] = mapIt->second.m_enqueueTime;
        // }
        // mapIt->second.m_holtime = m_lastdequeueTimeMap[mapIt->second.m_srcNodeId][mapIt->second.m_successLinkId];
        //m_lastdequeueTimeMap[mapIt->second.m_srcNodeId][mapIt->second.m_successLinkId] = mapIt->second.m_dequeueTime;
        if (m_statsCollecting)
        {
            if (mapIt->second.m_acked)
            {
                //Put record into success map and remove it from inflight map
                for(const auto & srcmap : m_successMap)
                {
                    if(srcmap.first.first != (uint32_t)mapIt->second.m_srcNodeId) continue;
                    for (const auto & mitmap: srcmap.second)
                    {
                        uint32_t dep=0;
                        for(auto mit = mitmap.second.rbegin();mit!=mitmap.second.rend();mit++)
                        {
                            mapIt->second.m_holtime = (mit->m_dequeueTime > mapIt->second.m_holtime && 
                                                    mit->m_dequeueTime != mapIt->second.m_dequeueTime) 
                                                        ? mit->m_dequeueTime : mapIt->second.m_holtime;
                            if(dep++>1000) break;
                        }
                    }
                }
                for(const auto &srcmap : m_failureMap)
                {
                    if(srcmap.first.first != (uint32_t)mapIt->second.m_srcNodeId) continue;
                    for (const auto & mit: srcmap.second)
                    {
                        mapIt->second.m_holtime = (mit.m_dequeueTime > mapIt->second.m_holtime) ? mit.m_dequeueTime : mapIt->second.m_holtime;
                    }
                }
                // std::cout<<"Success Packet: toNode:"<<mapIt->second.m_dstNodeId
                // <<" enqueue: "<<mapIt->second.m_enqueueTime.ToDouble(Time::MS)
                // <<" txStart: "<<mapIt->second.m_txStartTime.ToDouble(Time::MS)
                // <<" holtime: "<<mapIt->second.m_holtime.ToDouble(Time::MS)
                // <<" ackTime: "<<mapIt->second.m_ackTime.ToDouble(Time::MS)
                // <<" dequeueTime: "<<mapIt->second.m_dequeueTime.ToDouble(Time::MS)
                // <<" queueing: "<<(mapIt->second.m_holtime-mapIt->second.m_enqueueTime).ToDouble(Time::MS)
                // <<"\n"<<std::endl;
                //std::cout<<(mapIt->second.m_holtime-mapIt->second.m_enqueueTime).ToDouble(Time::MS)<<" "<<(mapIt->second.m_dequeueTime-mapIt->second.m_holtime).ToDouble(Time::MS)<<std::endl;
                m_successMap[{mapIt->second.m_srcNodeId,mapIt->second.m_dstNodeId}][mapIt->second.m_successLinkId].emplace_back(mapIt->second);
            }
            else
            {
                // std::cout<<"Fail Packet: toNode:"<<mapIt->second.m_dstNodeId
                // <<" enqueue: "<<mapIt->second.m_enqueueTime.ToDouble(Time::MS)
                // <<" txStart: "<<mapIt->second.m_txStartTime.ToDouble(Time::MS)
                // <<" holtime: "<<mapIt->second.m_holtime.ToDouble(Time::MS)
                // <<" queueing: "<<(mapIt->second.m_holtime-mapIt->second.m_enqueueTime).ToDouble(Time::MS)
                // <<"\n"<<std::endl;

                m_failureMap[{mapIt->second.m_srcNodeId,mapIt->second.m_dstNodeId}].emplace_back(mapIt->second);
            }
        }
        m_inflightMap.erase(mapIt);
    }
}

void 
WifiTxStatsPlotHelper::GenerateAllDelays(std::vector<WifiTxPerPktRecord> records,std::string outfilename)
{
    std::sort(records.begin(),records.end(),[](const WifiTxPerPktRecord & a,const WifiTxPerPktRecord & b){return a.m_ackTime-a.m_enqueueTime < b.m_ackTime-b.m_enqueueTime;});
    std::ofstream outFile((outfilename+".plt").c_str(), std::ios::out);
    Gnuplot plot (outfilename+".png");
    plot.SetTitle("delay of each packet");
    plot.SetTerminal("png");
    plot.SetLegend("index", "delay");
    plot.AppendExtra("set xrange [0:1600]");
    plot.AppendExtra("set yrange [0:20]");
    Gnuplot2dDataset e2edataset,transmitdataset,queuedataset;
    e2edataset.SetTitle("delay");
    transmitdataset.SetTitle("access");
    queuedataset.SetTitle("queue");
    size_t total_packet = records.size();
    for (size_t index=0;index<total_packet;index++)
    {
        e2edataset.Add(index,(records[index].m_ackTime-records[index].m_enqueueTime).ToDouble(Time::MS));
        transmitdataset.Add(index,(records[index].m_ackTime-records[index].m_holtime).ToDouble(Time::MS));
        queuedataset.Add(index,(records[index].m_holtime-records[index].m_enqueueTime).ToDouble(Time::MS));
    }
    plot.AddDataset(e2edataset);
    plot.AddDataset(transmitdataset);
    plot.AddDataset(queuedataset);
    plot.GenerateOutput(outFile);
    outFile.close();
}

void 
WifiTxStatsPlotHelper::GenerateCDF(std::vector<WifiTxPerPktRecord> records,std::string outfilename)
{
    
    std::ofstream outFile((outfilename+".plt").c_str(), std::ios::out);
    Gnuplot plot (outfilename+".png");
    plot.SetTitle("CDF of UL");
    plot.SetTerminal("png");
    plot.SetLegend("delay", "f");
    plot.AppendExtra("set xrange [0:1]");
    plot.AppendExtra("set yrange [0:1]");
    
    Gnuplot2dDataset datasete2e,datasetaccess,datasetqueue;
    datasete2e.SetTitle("e2e");
    datasetaccess.SetTitle("access");
    datasetqueue.SetTitle("queue");
    size_t total_packet = records.size();
    std::sort(records.begin(),records.end(),[](const WifiTxPerPktRecord & a,const WifiTxPerPktRecord & b){return a.m_ackTime-a.m_enqueueTime < b.m_ackTime-b.m_enqueueTime;});
    for (size_t index=0;index<total_packet;index++)
    {
        datasete2e.Add((records[index].m_ackTime-records[index].m_enqueueTime).ToDouble(Time::MS),double(index)/double(total_packet));
    }
    std::sort(records.begin(),records.end(),[](const WifiTxPerPktRecord & a,const WifiTxPerPktRecord & b){return a.m_ackTime-a.m_holtime < b.m_ackTime-b.m_holtime;});
    for (size_t index=0;index<total_packet;index++)
    {
        datasetaccess.Add((records[index].m_ackTime-records[index].m_holtime).ToDouble(Time::MS),double(index)/double(total_packet));
    }
    std::sort(records.begin(),records.end(),[](const WifiTxPerPktRecord & a,const WifiTxPerPktRecord & b){return a.m_holtime-a.m_enqueueTime < b.m_holtime-b.m_enqueueTime;});
    for (size_t index=0;index<total_packet;index++)
    {
        datasetqueue.Add((records[index].m_holtime-records[index].m_enqueueTime).ToDouble(Time::MS),double(index)/double(total_packet));
    }
    plot.AddDataset(datasete2e);
    plot.AddDataset(datasetaccess);
    plot.AddDataset(datasetqueue);
    plot.GenerateOutput(outFile);
    outFile.close();
}
}
