/*
 * SPDX-License-Identifier: GPL-2.0-only
 */

// Network topology
//
//       n0    n1   n2   n3
//       |     |    |    |
//     =====================
//
//  node n0,n1,n3 pings to node n2
//  node n0 generates protocol 2 (IGMP) to node n3

#include "ns3/applications-module.h"
#include "ns3/core-module.h"
#include "ns3/csma-module.h"
#include "ns3/internet-apps-module.h"
#include "ns3/internet-module.h"
#include "ns3/network-module.h"

#include <cassert>
#include <fstream>
#include <iostream>
#include <string>

using namespace ns3;

NS_LOG_COMPONENT_DEFINE("CsmaPingExample");

/**
 * Rx sink
 *
 * \param p The packer.
 * \param ad The sender address.
 */
static void
SinkRx(Ptr<const Packet> p, const Address& ad)
{
    std::cout << *p << std::endl;
}

/**
 * Ping RTT trace sink
 *
 * \param context The context.
 * \param seqNo The Sequence Number.
 * \param rtt The RTT.
 */
static void
PingRtt(std::string context, uint16_t seqNo, Time rtt)
{
    std::cout << context << " " << seqNo << " " << rtt << std::endl;
}

int
main(int argc, char* argv[])
{
    bool verbose{false};

    CommandLine cmd(__FILE__);
    cmd.AddValue("verbose",
                 "print received background traffic packets and ping RTT values",
                 verbose);
    cmd.Parse(argc, argv);

    // Here, we will explicitly create four nodes.
    NS_LOG_INFO("Create nodes.");
    NodeContainer c;
    c.Create(4);

    // connect all our nodes to a shared channel.
    NS_LOG_INFO("Build Topology.");
    CsmaHelper csma;
    csma.SetChannelAttribute("DataRate", DataRateValue(DataRate(5000000)));
    csma.SetChannelAttribute("Delay", TimeValue(MilliSeconds(2)));
    csma.SetDeviceAttribute("EncapsulationMode", StringValue("Llc"));
    NetDeviceContainer devs = csma.Install(c);

    // add an ip stack to all nodes.
    NS_LOG_INFO("Add ip stack.");
    InternetStackHelper ipStack;
    ipStack.Install(c);

    // assign ip addresses
    NS_LOG_INFO("Assign ip addresses.");
    Ipv4AddressHelper ip;
    ip.SetBase("192.168.1.0", "255.255.255.0");
    Ipv4InterfaceContainer addresses = ip.Assign(devs);

    NS_LOG_INFO("Create Source");
    Config::SetDefault("ns3::Ipv4RawSocketImpl::Protocol", StringValue("2"));
    InetSocketAddress dst(addresses.GetAddress(3));
    OnOffHelper onoff = OnOffHelper("ns3::Ipv4RawSocketFactory", dst);
    onoff.SetConstantRate(DataRate(15000));
    onoff.SetAttribute("PacketSize", UintegerValue(1200));

    ApplicationContainer apps = onoff.Install(c.Get(0));
    apps.Start(Seconds(1.0));
    apps.Stop(Seconds(10.0));

    NS_LOG_INFO("Create Sink.");
    PacketSinkHelper sink = PacketSinkHelper("ns3::Ipv4RawSocketFactory", dst);
    apps = sink.Install(c.Get(3));
    apps.Start(Seconds(0.0));
    apps.Stop(Seconds(11.0));

    NS_LOG_INFO("Create pinger");
    PingHelper ping(addresses.GetAddress(2));
    NodeContainer pingers;
    pingers.Add(c.Get(0));
    pingers.Add(c.Get(1));
    pingers.Add(c.Get(3));
    apps = ping.Install(pingers);
    apps.Start(Seconds(2.0));
    apps.Stop(Seconds(5.0));

    NS_LOG_INFO("Configure Tracing.");
    // first, pcap tracing in non-promiscuous mode
    csma.EnablePcapAll("csma-ping", false);

    if (verbose)
    {
        // then, print what the packet sink receives.
        Config::ConnectWithoutContext("/NodeList/3/ApplicationList/0/$ns3::PacketSink/Rx",
                                      MakeCallback(&SinkRx));
        // finally, print the ping rtts.
        Config::Connect("/NodeList/*/ApplicationList/*/$ns3::Ping/Rtt", MakeCallback(&PingRtt));
    }

    Packet::EnablePrinting();

    NS_LOG_INFO("Run Simulation.");
    Simulator::Run();
    Simulator::Destroy();
    NS_LOG_INFO("Done.");

    return 0;
}
