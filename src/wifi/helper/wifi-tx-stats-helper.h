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

#ifndef WIFI_TX_STATS_HELPER_H
#define WIFI_TX_STATS_HELPER_H

#include <cstdint>
#include <map>
#include <vector>

#include <ns3/ptr.h>
#include <ns3/object.h>
#include <ns3/type-id.h>
#include <ns3/qos-utils.h>
#include <ns3/nstime.h>
#include <ns3/node-container.h>

namespace ns3
{

using NodeIdPair = std::pair<uint32_t, uint32_t>;
// The final result
struct WifiTxFinalStatistics
{
    std::map<NodeIdPair /* Node ID */,
             std::map<uint8_t /* Link ID */, uint64_t>> m_numSuccessPerNodePairLink;

    std::map<NodeIdPair /* Node ID */,
    std::map<uint8_t /* Link ID */, uint64_t>> m_successSizePerNodePairLink;
    std::map<NodeIdPair /* Node ID */, uint64_t> m_numRetransmittedPerNodePair; // For success pkts only
    std::map<NodeIdPair /* Node ID */, double> m_avgFailuresPerNodePair; // For success pkts only
    std::map<NodeIdPair /* Node ID */, uint64_t> m_numFinalFailedPerNodePair;
    std::map<NodeIdPair /* Node ID */,
             std::map<uint8_t /* Link ID */, double>> m_meanE2eDelayPerNodePairLink; // 每组连接之间的端到端时延ms
    std::map<NodeIdPair /* Node ID */,
             std::map<uint8_t /* Link ID */, double>> m_099E2eDelayPerNodePairLink; // 每组连接之间的99%端到端时延ms
    std::map<NodeIdPair /* Node ID */,
             std::map<uint8_t /* Link ID */, double>> m_095E2eDelayPerNodePairLink; // 每组连接之间的95%端到端时延ms
    std::map<NodeIdPair /* Node ID */,
             std::map<uint8_t /* Link ID */, double>> m_meanAccessDelayPerNodePairLink; // 每组连接之间的接入时延ms
    std::map<NodeIdPair /* Node ID */,
             std::map<uint8_t /* Link ID */, double>> m_099AccessDelayPerNodePairLink; // 每组连接之间的99%接入时延ms
    std::map<NodeIdPair /* Node ID */,
             std::map<uint8_t /* Link ID */, double>> m_095AccessDelayPerNodePairLink; // 每组连接之间的95%接入时延ms
    std::map<NodeIdPair /* Node ID */,
             std::map<uint8_t /* Link ID */, double>> m_meanQueueDelayPerNodePairLink; // 每组连接之间的排队时延ms
    std::map<NodeIdPair /* Node ID */,
             std::map<uint8_t /* Link ID */, double>> m_099QueueDelayPerNodePairLink; // 每组连接之间的99%排队时延ms
    std::map<NodeIdPair /* Node ID */,
             std::map<uint8_t /* Link ID */, double>> m_095QueueDelayPerNodePairLink; // 每组连接之间的95%排队时延ms
    double m_meanE2eDelayTotal; // millisecond
    uint64_t m_numSuccessTotal;
    uint64_t m_numRetransmittedTotal; // For success pkts only
    double m_avgFailuresTotal; // For success pkts only
    uint64_t m_numFinalFailedTotal;
    WifiTxFinalStatistics() :
          m_meanE2eDelayTotal(0), m_numSuccessTotal(0), m_numRetransmittedTotal(0),
          m_avgFailuresTotal(0.0), m_numFinalFailedTotal(0)
    {

    }
};

// Per-packet record, created when enqueued at MAC layer
struct WifiTxPerPktRecord
{
    int m_srcNodeId;
    int m_dstNodeId;
    int m_seqNum;
    int m_tid;
    int m_successLinkId;
    uint8_t m_failures;
    Time m_enqueueTime;
    Time m_txStartTime;
    Time m_txEndTime;
    Time m_holtime;
    Time m_ackTime;
    Time m_dequeueTime;
    bool m_txStarted;
    bool m_acked;
    bool m_dequeued;
    uint32_t m_packetSize;
    WifiTxPerPktRecord() :
          m_srcNodeId(-1),m_dstNodeId(-1), m_seqNum(-1), m_tid(-1), m_successLinkId(-1), m_failures(0),
          m_enqueueTime(0), m_txStartTime(0), m_txEndTime(0), m_holtime(0), m_ackTime(0), m_dequeueTime(0), 
          m_txStarted(false), m_acked(false), m_dequeued(false)
    {}
};
typedef std::map<NodeIdPair /* Node ID */, std::map<uint8_t /* Link ID */, std::vector<WifiTxPerPktRecord>>> WifiPktTxRecordMap;
typedef std::map<uint64_t /* UID */, WifiTxPerPktRecord> WifiPktUidMap;
typedef std::map<NodeIdPair /* Node ID */, std::vector<WifiTxPerPktRecord>> WifiPktNodeIdMap;
typedef std::map<NodeIdPair /* Node ID */, std::map<uint8_t /* Link ID */, Time>> WifiLastTimeMap;

// Forward declaration
class NetDeviceContainer;
class Time;
class Packet;
class WifiMpdu;
class WifiTxStatsTraceSink;

class WifiTxStatsHelper
{
  public:
    WifiTxStatsHelper();
    void Enable(NetDeviceContainer devices);
    WifiTxFinalStatistics GetFinalStatistics();
    std::vector<WifiTxPerPktRecord> GetSuccessPktsTotal();
    std::vector<WifiTxPerPktRecord> GetSuccessPktsPerNode(NodeIdPair nodepair);
    const WifiPktTxRecordMap& GetSuccessInfoMap();
    const WifiPktNodeIdMap& GetFailureInfoMap();
    void Start(Time startTime);
    void Stop(Time stopTime);
    void Reset();
    const std::map<AcIndex, std::string> m_aciToString = {
        {AC_BE, "BE"},
        {AC_BK, "BK"},
        {AC_VI, "VI"},
        {AC_VO, "VO"},
    };

  private:
    Ptr<WifiTxStatsTraceSink> m_traceSink;
    NodeContainer global_nodes;
    std::map<Mac48Address,uint32_t /*Node ID*/> MacAddresstoNodeId;
};

class WifiTxStatsPlotHelper
{
  public:
    WifiTxStatsPlotHelper()
    {
    }
    static void GenerateAllDelays(std::vector<WifiTxPerPktRecord> records,std::string outfilename);
    static void GenerateCDF(std::vector<WifiTxPerPktRecord> records,std::string outfilename);

};

class WifiTxStatsTraceSink : public Object
{
  public:
    WifiTxStatsTraceSink();
    static TypeId GetTypeId();
    void DoStart();
    void DoStop();
    void DoReset();
    WifiTxFinalStatistics DoGetFinalStatistics(); 
    const WifiPktTxRecordMap& DoGetSuccessInfoMap();
    const WifiPktNodeIdMap& DoGetFailureInfoMap();

    // functions to be called back
    void NotifyMacEnqueue(Ptr<const WifiMpdu> mpdu);
    void NotifyTxStart(Ptr<const Packet> pkt, double txPowerW);
    void NotifyTxEnd(Ptr<const Packet> pkt);
    void NotifyAcked(Ptr<const WifiMpdu> mpdu, uint8_t linkId);
    void NotifyMacDequeue(Ptr<const WifiMpdu> mpdu);
    void SetMacAddresstoNodeId(std::map<Mac48Address,uint32_t> MacAddresstoNodeId)
    {
        this->MacAddresstoNodeId = MacAddresstoNodeId;
    }
  private:
    bool m_statsCollecting;
    WifiPktUidMap m_inflightMap;
    WifiPktTxRecordMap m_successMap;
    WifiPktNodeIdMap m_failureMap;
    WifiLastTimeMap m_lastdequeueTimeMap;
    std::map<Mac48Address,uint32_t /*Node ID*/> MacAddresstoNodeId;
};

}

#endif 
