
/*
 * Code structure
 * N  node containers
 * N wifi devices, mobility models
 *
 *New class for application
 *
 */

/* Code to understand the effect of performance on
 * many number of nodes in a given space.
 * affect on throughput and latency is of interest for now
 */

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

// global variables
/*
//unsigned int seqnum=0;
double TargetThroughput =2; // mbps
double throughput=0.0;
double latency=0.0;
unsigned int packetSize=1024;
unsigned int counter=0;
uint32_t numPackets = 1000;
double interval = packetSize*8.0/(TargetThroughput*1000000); // seconds
//double interval = 1.0;
*/

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
	//void ReceivePacket(Ptr<Socket> socket);
	void GenerateTraffic (int);
	//void ScheduleSimulation();



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

//void perfTestApp::ReceivePacket(Ptr<Socket> socket)
void ReceivePacket(Ptr<Socket> socket)
{

//double lossRate;
//double totalLatency;
//long totalBytes;
//double totalTime;
//long numberReceived;
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
	 // printf("%lld,%lld,%d\n",Now ().GetMicroSeconds (),seqTs.GetTs ().GetMicroSeconds (),pkt->GetSize());
 // double lat= Now ().GetMicroSeconds ()- seqTs.GetTs ().GetMicroSeconds ();
	  //cout<<lat<<endl;
	  //latency += lat;
	  //double thr = pkt->GetSize()*8.0/lat;
	  //cout << thr <<" ... "<< slat << "  "<< pkt->GetSize() << "...." << seqTs.GetSeq()<<endl;

      //throughput += thr;
	  //counter++;
	  numberReceived++;
	  totalLatency+=Now().GetMicroSeconds()-seqTs.GetTs().GetMicroSeconds();
	 // totalTime+=Now().GetMicroSeconds();
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

		//if(nodeId == 0) //  only print for sending node
		fprintf(resfpt,"[%d], %lld, %d, %d\n",nodeId,seqTs.GetTs().GetMicroSeconds(),seqTs.GetSeq(),p->GetSize());

	      //Ptr <MobilityModel> pos = this->sendSocket->GetNode()->GetObject<MobilityModel>();
	      //pos->SetPosition(Vector(10,0,0));

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
	//int ScheduleSimulation(unsigned int nodeID);
	int createSendSocket(unsigned int nodeID, int);
	int createRecvSocket(unsigned int nodeID, int);
	Ptr<Node> getNodePtr();
	NodeContainer getNode();


	perfTestApp *apps;

	unsigned int N;
	NodeContainer node;
	NetDeviceContainer device;
	//Ptr<Socket> recvSocket;
	//Ptr<Socket> sendSocket;
	double throughput;
	unsigned int numberOfPacket;
	unsigned int packetSize;
	unsigned int seqNum;
	unsigned int counter;
	unsigned int packetCount;
	Time interPacketInterval;
	//InetSocketAddress local;
	//InetSocketAddress remote;
	//FILE *fp;
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
//	fclose(fp);

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
	 // cout<<apps[nodeID].sendSocket->GetNode()->GetId()<<"  "<<Now().GetMicroSeconds()<<endl;
	  //pps[nodeID].sendSocket=sendSocket;
	  //cout << "done creating send socket\n";
	  return 0;
}

int perfTestNodes::createRecvSocket(unsigned int nodeID, int port)
{
	  TypeId tid = TypeId::LookupByName ("ns3::UdpSocketFactory");
	  apps[nodeID].recvSocket = Socket::CreateSocket (node.Get (nodeID), tid);
	  InetSocketAddress local = InetSocketAddress (Ipv4Address::GetAny (), port);
	  apps[nodeID].recvSocket->Bind (local);
	  //recvSocket->SetRecvCallback (MakeCallback(&perfTestNodes::ReceivePacket,this));
	  apps[nodeID].recvSocket->SetRecvCallback (MakeCallback(&(ReceivePacket)));
	  //recvSocket->SetRecvCallback(MakeCallback(&perfTestNodes::ReceivePacket,this));
	  //cout<<apps[nodeID].recvSocket->GetNode()->GetId()<<"  "<<Now().GetMicroSeconds()<<endl;
	 // cout<<"create recv socket"<<endl;
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
	//cout<<interval<<"\t"<<interPacketInterval<<endl;
	seqNum=0;
	packetCount=numberOfPacket;
	//fp=fopen("SST.dat","w");

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
  	// mobility.SetMobilityModel ("ns3::ConstantPositionMobilityModel");
   	m.Install (n);

}




void perfTestNodes::setupNode(unsigned int N1,char *pm,char* size) //create 1 node
{
    //remote=InetSocketAddress (Ipv4Address::GetAny (), 80);
    //local=InetSocketAddress (Ipv4Address::GetAny (), 80);
    //
       char str[200];
       char* str1 ="ns3::UniformRandomVariable[Min=0.0|Max=\0";
       strcpy(str,str1);
	strcat(strcat(str,size),"]\0"); 
	N=N1+1; // set up one node sas a snifer 
	//node =new NodeContainer[N];
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
	//mobility.SetPositionAllocator("ns3::RandomRectanglePositionAllocator","X",StringValue("ns3::UniformRandomVariable[Min=0.0|Max=50.0]"),"Y",StringValue("ns3::UniformRandomVariable[Min=0.0|Max=50.0]"));
	//mobility.SetMobilityModel ("ns3::RandomWalk2dMobilityModel",
	 //                                "Bounds", RectangleValue (Rectangle (0.0, 100.0, 0.0, 100.0)),"Distance",DoubleValue(100.0),"Speed",StringValue("ns3::UniformRandomVariable[Min=25.0|Max=25.0]")); //mode distance, speed 25
	mobility.SetMobilityModel ("ns3::ConstantPositionMobilityModel");
	mobility.Install (node);

	InternetStackHelper internet;
	internet.Install (node);

	//cout<<"done setting up nodes"<<endl;
}

void ScheduleSimulation(perfTestNodes *p,unsigned int nodeID)
{
	//Simulator::Schedule (Seconds(x), &perfTestNodes::GenerateTraffic,  this);
	if (p->apps[nodeID].packetCount > 0)
		    {

		      p->apps[nodeID].GenerateTraffic(nodeID);
		      p->apps[nodeID].seqNum++;
		      p->apps[nodeID].packetCount--;
		      //cout<<seqNum<<endl;
		      //cout<<p->apps[nodeID].sendSocket->GetNode()->GetId()<<"  "<<p->apps[nodeID].packetSize<<" "<<Now().GetMicroSeconds()<<"\n";
		      Simulator::Schedule (Seconds(p->apps[nodeID].interPacketInterval),&ScheduleSimulation,p,nodeID);
		    }
		else
		    {
		      p->apps[nodeID].sendSocket->Close ();
		    }
}


int main(int argc, char *argv[])
{
	/*
	 * Argument list
	 *  ::  randomnumber seed
	 *  :: numberofnodes
	 *  :: daterate
	 *  :: packetsize
	 *  :: ite ration
	 */
//double lossRate;
//double totalLatency;
//long totalBytes;
//double totalTime;
//long numberReceived;

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
	unsigned int ATTACKER;
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
	//for(i=0;i<N;i++)
	//	TotalNodes[i].setupNode(&nodes[i]);
	
	//maxx=maxX;
	//maxy=maxY;

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
	ATTACKER=1;
	
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
	//printf("before doing setupnode\n");
   	TotalNodes.setupNode(N,physicalMode,size);
	//printf("after doing that damn thing\n");

	//NodeContainer AllNodes;


	// add two nodes at fixed distance 
	

	
	TotalNodes.setNodeLocation(SENDER,10.0,s/2,0.0);
	TotalNodes.setNodeLocation(RECEIVER,10+distance,s/2,0.0);
	TotalNodes.setNodeLocation(SNIFFER,s/2,s/2,0);
	TotalNodes.setNodeLocation(ATTACKER,10+distance/2,s/2,0.0);
//	TotalNodes.setNodeLocation(SNIFFER,s/2,s/2,0);


	TotalNodes.packetConfig(PktSize,iteration,thrpt/1000000);
	
	// create attack node parameters, thrpt= 30 mbps, pkt size =1000, count = 1000000;
	
	TotalNodes.apps[ATTACKER].seqNum=0;
	TotalNodes.apps[ATTACKER].interPacketInterval=1000*8.0/(60*1000000);
	TotalNodes.apps[ATTACKER].packetCount=100000;
	TotalNodes.apps[ATTACKER].packetSize=1000;
	


	Ipv4AddressHelper ipv4;
	//  NS_LOG_INFO ("Assign IP Addresses.");
	ipv4.SetBase ("10.1.0.0", "255.255.0.0");
	TotalNodes.addIPAddress(&ipv4);

	//TotalNodes.addIPAddress(SENDER,&ipv4);
	TotalNodes.createSendSocket(SENDER,81);

	for(i=1;i<N-1;i++)
	{
		//TotalNodes.addIPAddress(i,&ipv4);
		TotalNodes.createSendSocket(i,80);
	}

	//TotalNodes.addIPAddress(N-1,&ipv4);
	TotalNodes.createRecvSocket(RECEIVER,81);

	//for(i=0;i<N;i++)
	//		AllNodes.Add(TotalNodes[i].getNode());
	Simulator::Schedule (Seconds(0.0001),ScheduleSimulation,&TotalNodes,0);

	for(i=1;i<N-1;i++){
		
		//TotalNodes.ScheduleSimulation(i,i*1.0/100);
		Simulator::Schedule (Seconds(i/1000.0),ScheduleSimulation,&TotalNodes,i);
		//Simulator::Schedule(Seconds(i*1.0/100),&perfTestApp::ScheduleSimulation);
		//AllNodes.Get(i)
		//Simulator::ScheduleWithContext (TotalNodes.apps[i].sendSocket->GetNode ()->GetId (),
		    //                        Seconds (i*1.0/100), &TotalNodes.apps[i].ScheduleSimulation);
	}
	//AnimationInterface anim (animFile);
	//Simulator::Stop (Seconds (10.0));
	//
	totalBytes=0;
	totalTime=0;


//double lossRate;
//double totalLatency;
///long totalBytes;
//double totalTime;
//long numberReceived;

	startTime=Now().GetMicroSeconds();

	Simulator::Run ();
	//Simulator::Destroy ();
	endTime=Now().GetMicroSeconds();
	Simulator::Destroy();
	
	cout<<"Total simulation time = "<<(endTime-startTime)/1000000.0<<" seconds"<<endl;	

	//cout<<"lossrate =  "<<(1.0-numberReceived*1.0/iteration)<<"   average latency=  "<<totalLatency*1.0/numberReceived<<"  average throughput == "<< totalBytes*8*1000000.0/(endTime-startTime)<<"  arrival Rate = "<<numberReceived*1000000.0/(endTime-startTime)<<endl;
	cout<<"pkt error rate = "<<(1.0-numberReceived*1.0/iteration)<<"\n"<<"average latency  = "<<totalLatency*1.0/numberReceived<<"microseconds"<<"\n"<<"throughput mbps =  "<<totalBytes*8.0/(endTime-startTime)<<"\n"<<"packets per second  =  "<<numberReceived*1000000.0/(endTime-startTime)<<endl;
	//cout<<"Simulation: done"<<endl;
	//printf("Simulation Done\n");
	fclose(resfpt);
	fclose(resfpr);
	return 0;
}







