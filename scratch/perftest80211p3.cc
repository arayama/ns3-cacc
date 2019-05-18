

#include "ns3/vector.h"
#include "ns3/string.h"
#include "ns3/socket.h"
#include "ns3/double.h"
#include "ns3/config.h"
#include "ns3/log.h"
#include "ns3/command-line.h"
#include "ns3/mobility-model.h"
#include "ns3/yans-wifi-helper.h"
#include "ns3/position-allocator.h"
#include "ns3/mobility-helper.h"
#include "ns3/internet-stack-helper.h"
#include "ns3/ipv4-address-helper.h"
#include "ns3/ipv4-interface-container.h"
#include <iostream>

#include "ns3/ocb-wifi-mac.h"
#include "ns3/wifi-80211p-helper.h"
#include "ns3/wave-mac-helper.h"
#include "ns3/seq-ts-header.h"
#include "ns3/netanim-module.h"
#include "ns3/rng-seed-manager.h"
#include "ns3/snr-tag.h"

#define verbose false
#define maxX 100.0
#define maxY 100.0
#define minX 0.0
#define minY 0.0

using namespace ns3;
using namespace std;

NS_LOG_COMPONENT_DEFINE ("perfTestLog");

FILE* resfpt; // pointer to transmit file 
FILE* resfpr; //pointer to receive file 
char sniff[200];
double lossRate;
double totalLatency;
long totalBytes;
double totalTime;
long numberReceived;
uint64_t startTime;
uint64_t endTime;

class perfTestApp
{
public:
	perfTestApp();
	~perfTestApp();
	void GenerateTraffic (int);



	Ptr<Socket> recvSocket;
	Ptr<Socket> sendSocket;

	unsigned int packetCount;
	unsigned int seqNum;
	double interPacketInterval;
	unsigned int packetSize;
};

perfTestApp::perfTestApp()
{


}

perfTestApp::~perfTestApp()
{

}

void ReceivePacket(Ptr<Socket> socket)
{

	  SnrTag tag;
	  Ptr<Packet> pkt;
	  double temp_snr=9999;
	 // pkt =  recvSocket->Recv ();
	  pkt = socket->Recv();
	 // NS_LOG_UNCOND ("Received one packet!");
	  SeqTsHeader seqTs;
	  pkt->PeekHeader (seqTs);
	  if (pkt->PeekPacketTag(tag)){
	        temp_snr = tag.Get();
	  }
      fprintf(resfpr,"%lld,%lld,%d,%d,%f\n",Now ().GetMicroSeconds (),seqTs.GetTs ().GetMicroSeconds (),seqTs.GetSeq(),pkt->GetSize(),temp_snr);

	  numberReceived++;
	  totalLatency+=Now().GetMicroSeconds()-seqTs.GetTs().GetMicroSeconds();
	  totalBytes+=pkt->GetSize();
}

void perfTestApp::GenerateTraffic(int nodeId)
{

	     
	      //int n = this->GetId();
	      Ptr<Packet> p  = Create<Packet> (packetSize);
	      SeqTsHeader seqTs;
	      seqTs.SetSeq (this->seqNum);
	      p->AddHeader (seqTs);

	      this->sendSocket->Send (p);

		fprintf(resfpt,"[%d], %lld, %d, %d\n",nodeId,seqTs.GetTs().GetMicroSeconds(),seqTs.GetSeq(),p->GetSize());


}


class perfTestNodes
{
public:
	perfTestNodes();
	perfTestNodes(unsigned int N1);
	~perfTestNodes();

	void setupNode(unsigned int N1,char *pm,char* size);
	void setNodeLocation(unsigned int nodeID,float x,float y,float  z);
	int packetConfig(uint32_t packetSize1,uint32_t numberOfPacket1, double TargetThroughput1);
	int addIPAddress( Ipv4AddressHelper *ip);
	int createSendSocket(unsigned int nodeID, int);
	int createRecvSocket(unsigned int nodeID, int);
	Ptr<Node> getNodePtr();
	NodeContainer getNode();


	perfTestApp *apps;

	unsigned int N;
	NodeContainer node;
	NetDeviceContainer device;

	double throughput;
	unsigned int numberOfPacket;
	unsigned int packetSize;
	unsigned int seqNum;
	unsigned int counter;
	unsigned int packetCount;
	Time interPacketInterval;

};

Ptr<Node> perfTestNodes::getNodePtr()
{
	Ptr<Node> p = node.Get(0);
	return p;
}

NodeContainer perfTestNodes::getNode()
{
	return node;
}



perfTestNodes::perfTestNodes()
{


}

perfTestNodes::perfTestNodes(unsigned int N1)
{
	N=N1;

}

perfTestNodes::~perfTestNodes()
{


}




int perfTestNodes::addIPAddress(Ipv4AddressHelper *ip)
{

	Ipv4InterfaceContainer i = ip->Assign (device);
	return 0;
}

int perfTestNodes::createSendSocket(unsigned int nodeID, int port)
{
	  TypeId tid = TypeId::LookupByName ("ns3::UdpSocketFactory");
	  apps[nodeID].sendSocket = Socket::CreateSocket (node.Get (nodeID), tid);
	  InetSocketAddress remote = InetSocketAddress (Ipv4Address ("255.255.255.255"),port);
	  apps[nodeID].sendSocket->SetAllowBroadcast (true);
	  apps[nodeID].sendSocket->Connect (remote);

	  return 0;
}

int perfTestNodes::createRecvSocket(unsigned int nodeID, int port)
{
	  TypeId tid = TypeId::LookupByName ("ns3::UdpSocketFactory");
	  apps[nodeID].recvSocket = Socket::CreateSocket (node.Get (nodeID), tid);
	  InetSocketAddress local = InetSocketAddress (Ipv4Address::GetAny (), port);
	  apps[nodeID].recvSocket->Bind (local);
	  apps[nodeID].recvSocket->SetRecvCallback (MakeCallback(&(ReceivePacket)));

	  return 0;
}

int perfTestNodes::packetConfig(uint32_t packetSize1,uint32_t numberOfPacket1, double TargetThroughput1)
{
	packetSize=packetSize1;
	numberOfPacket=numberOfPacket1;
	throughput = TargetThroughput1; //mbps
	counter=0;
	double interval = packetSize*8.0/(throughput*1000000); // seconds
	interPacketInterval = Seconds (interval);
	seqNum=0;
	packetCount=numberOfPacket;

	for(int i=0;i<N;i++)
	{	apps[i].seqNum=0;
		apps[i].interPacketInterval=interval;
		apps[i].packetCount=packetCount;
		apps[i].packetSize=packetSize;
	}
	return 0;

}

void perfTestNodes::setNodeLocation( unsigned int nodeID, float x, float y,float z)
{

	Ptr <Node> n = this->node.Get(nodeID);

	MobilityHelper m;
	Ptr<ListPositionAllocator> positionAlloc = CreateObject<ListPositionAllocator> ();
   	positionAlloc->Add (Vector (x,y,z));
   	m.SetPositionAllocator (positionAlloc);
   	m.Install (n);

}




void perfTestNodes::setupNode(unsigned int N1,char *pm,char* size) //create 1 node
{

       char str[200];
       char* str1 ="ns3::UniformRandomVariable[Min=0.0|Max=\0";
       strcpy(str,str1);
	strcat(strcat(str,size),"]\0"); 
	N=N1+1; // set up one node sas a snifer 
	std::string phyMode (pm);
	node.Create(N);

	apps=new perfTestApp[N];
	// The below set of helpers will help us to put together the wifi NICs we want

	YansWifiChannelHelper waveChannel = YansWifiChannelHelper (); // create channel without any config
	waveChannel.SetPropagationDelay("ns3::ConstantSpeedPropagationDelayModel");

    waveChannel.AddPropagationLoss("ns3::ThreeLogDistancePropagationLossModel","Distance0",DoubleValue(1.0),"Distance1",DoubleValue(210),"Distance2",DoubleValue(286),"Exponent0",DoubleValue(1.9),"Exponent1",DoubleValue(15),"Exponent2",DoubleValue(3.65));
    waveChannel.AddPropagationLoss("ns3::NakagamiPropagationLossModel","Distance1",DoubleValue(80.0),"Distance2",DoubleValue(320),"m0",DoubleValue(1.5),"m1",DoubleValue(0.75),"m2",DoubleValue(0));



	YansWifiPhyHelper wifiPhy =  YansWifiPhyHelper::Default ();
	//YansWifiChannelHelper wifiChannel = YansWifiChannelHelper::Default ();
	Ptr<YansWifiChannel> channel = waveChannel.Create ();
	wifiPhy.SetChannel (channel);
	// ns-3 supports generate a pcap trace
	wifiPhy.SetPcapDataLinkType (YansWifiPhyHelper::DLT_IEEE802_11);
	NqosWaveMacHelper wifi80211pMac = NqosWaveMacHelper::Default ();
	Wifi80211pHelper wifi80211p = Wifi80211pHelper::Default ();
	if (verbose)
	    {
	      wifi80211p.EnableLogComponents ();      // Turn on all Wifi 802.11p logging
	    }

	wifi80211p.SetRemoteStationManager ("ns3::ConstantRateWifiManager",
	                                      "DataMode",StringValue (phyMode),
	                                    "ControlMode",StringValue (phyMode),
					    "NonUnicastMode", StringValue (phyMode));


	device = wifi80211p.Install (wifiPhy, wifi80211pMac, node);
	//wifiPhy.EnablePcap (sniff,N-1,0,true);
	wifiPhy.EnablePcap(sniff,device.Get(N-1),true);
	MobilityHelper mobility;
	
	mobility.SetPositionAllocator("ns3::RandomRectanglePositionAllocator","X",StringValue(str),"Y",StringValue(str));
	mobility.SetMobilityModel ("ns3::ConstantPositionMobilityModel");
	mobility.Install (node);

	InternetStackHelper internet;
	internet.Install (node);

}

void ScheduleSimulation(perfTestNodes *p,unsigned int nodeID)
{
	if (p->apps[nodeID].packetCount > 0)
		    {

		      p->apps[nodeID].GenerateTraffic(nodeID);
		      p->apps[nodeID].seqNum++;
		      p->apps[nodeID].packetCount--;
		      Simulator::Schedule (Seconds(p->apps[nodeID].interPacketInterval),&ScheduleSimulation,p,nodeID);
		    }
		else
		    {
		      p->apps[nodeID].sendSocket->Close ();
		    }
}


int main(int argc, char *argv[])
{

	uint32_t SEED=1000; 
	uint64_t RUN;
	unsigned int N =2;
	int i=0;
	//resfp = fopen("SST.dat","w");
	unsigned int PktSize=200;
	double thrpt = 64000;
	unsigned int SENDER=0;
	unsigned int RECEIVER=N-1;
	unsigned int SNIFFER;
	//unsigned int RANDSEED ;
	float distance,s ;
	char size[10];
	char physicalMode[50];

	lossRate=0;
	totalLatency=0;
	totalBytes=0;
	totalTime=0;
	numberReceived=0;

	std::string animFile = "grid-animation.xml";
	//NodeContainer nodes[N];
        perfTestNodes TotalNodes;
	unsigned int iteration=1000;


	if(argc == 2) // only has random number seed
	{
		RUN=atoi(argv[1]);
	}
	else if (argc != 10)
	{       printf("Usage : perftest80211p2 RANDSEED numberofNodes datarate(kbps) packetsize(bytes) iteration size distance phymode filename");
		exit (1);
	}

	else
	{
		RUN=atoi(argv[1]);
		N = atoi(argv[2]);

		thrpt = atoi(argv[3])*1000; // kbps to bps 

		PktSize = atoi(argv[4]); // bytes 
		
		iteration = atoi(argv[5]); // total number of transmissions 

		strcpy(size,argv[6]);
		s=atof(argv[6]); //size in float 
		distance=atof(argv[7]);
		strcpy(physicalMode,argv[8]);

	}

	SENDER=0;
	RECEIVER=N-1;
	SNIFFER=N; 

	char ft[100],fr[100];
	strcpy(ft,argv[9]);
	strcpy(fr,argv[9]);
	strcpy(sniff,argv[9]);	
	strcat(fr,"_rec.dat");
	strcat(ft,"_tra.dat");
	strcat(sniff,"_sniff");	
	printf("%s \n %s\n %s\n",ft,ft,sniff);
	resfpt=fopen(ft,"w");
	resfpr=fopen(fr,"w");
	setvbuf( resfpt, (char *)NULL, _IONBF, 0 );

	setvbuf( resfpr, (char *)NULL, _IONBF, 0 );
	
	RngSeedManager::SetSeed (SEED); 
	RngSeedManager::SetRun (RUN);   
   	TotalNodes.setupNode(N,physicalMode,size);

	TotalNodes.setNodeLocation(SENDER,10.0,s/2,0.0);
	TotalNodes.setNodeLocation(RECEIVER,10+distance,s/2,0.0);
	TotalNodes.setNodeLocation(SNIFFER,s/2,s/2,0);


	TotalNodes.packetConfig(PktSize,iteration,thrpt/1000000);


	Ipv4AddressHelper ipv4;
	//  NS_LOG_INFO ("Assign IP Addresses.");
	ipv4.SetBase ("10.1.0.0", "255.255.0.0");
	TotalNodes.addIPAddress(&ipv4);

	TotalNodes.createSendSocket(SENDER,81);

	for(i=1;i<N-1;i++)
	{
		TotalNodes.createSendSocket(i,80);
	}

	TotalNodes.createRecvSocket(RECEIVER,81);

	Simulator::Schedule (Seconds(0.0001),ScheduleSimulation,&TotalNodes,0);

	for(i=1;i<N-1;i++){
			Simulator::Schedule (Seconds(i/1000.0),ScheduleSimulation,&TotalNodes,i);

	}
	//
	AnimationInterface anim (animFile);

	totalBytes=0;
	totalTime=0;

	startTime=Now().GetMicroSeconds();

	Simulator::Run ();
	endTime=Now().GetMicroSeconds();
	Simulator::Destroy();
	
	cout<<"Total simulation time = "<<(endTime-startTime)/1000000.0<<" seconds"<<endl;	

	cout<<"pkt error rate = "<<(1.0-numberReceived*1.0/iteration)<<"\n"<<"average latency  = "<<totalLatency*1.0/numberReceived<<"microseconds"<<"\n"<<"throughput mbps =  "<<totalBytes*8.0/(endTime-startTime)<<"\n"<<"packets per second  =  "<<numberReceived*1000000.0/(endTime-startTime)<<endl;

	fclose(resfpt);
	fclose(resfpr);
	return 0;
}







