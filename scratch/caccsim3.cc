
/*
 * Version 3.0 of the CACC simulation  in ns3 
 * Summary -- 
 * 	- gaussian noise added to distance and velocity measurements 
 * 	- kalman filter utilized  to filter the noise in  the measurements
 *  - Local acceleration updates using Lidar/RADAR  instead of DSRC 
 *    approximated based on the rate of change of distance  and  approach velocity 
 *
 *
 *
 * Code structure
 * N  node containers
 * N  wifi devices, mobility models
 * 
 * 
 *
 */
/***  info on kalman filter estimation technique 
 *  {rv,db} : velocity and distance between measurements coming from DME
 *  {dx,vx} :  velocity and distance travelled by veh_i
 *  {v=rv+vx, d=db+dx} ; inputs to Kf, z={v,d}
 *  (v_i)  = - (v_i-1) + 2 * d_i-1
 *  (d_i)  = (v_i-1) * delT + 1/2* (a_i-1) * delT^2
 *  z_i = x_i
 **********/
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
#include "ns3/random-variable-stream.h"
#include <iostream>

#include "ns3/ocb-wifi-mac.h"
#include "ns3/wifi-80211p-helper.h"
#include "ns3/wave-mac-helper.h"
#include "ns3/seq-ts-header.h"
#include "ns3/netanim-module.h"
#include "ns3/rng-seed-manager.h"
#include "ns3/snr-tag.h"

#include "packetTag.h"
#include  <math.h>
#include <arpa/inet.h>
#include <string.h>
#include <stdlib.h>
#include <stdio.h>
#include "gauss.h"


#define verbose false
#define m2ft 3.28084
#define maxacc 3.6576 // 3.6576 meters/s/s =   12 f/s/s 

#define ACC 0
#define CACC_U 1
#define CACC_DU 2
#define CACC_A 3 
#define CACC_RU 4
#define USE_UT true // if true, broadcasts u(t), if false use a(t) 


using namespace ns3;
using namespace std;
//using namespace rapidjson; 
NS_LOG_COMPONENT_DEFINE ("perfTestLog");



double distnNoise,velNoise; // standard deviation of zero mean gaussian noise process added as sensor noise 

// CACC global parameters 

double Kp=0.66;
double Kd=0.7;  //kd > kp * caccInterval
double h = 0.2;
double dref=10; //feet of distance between the vehicles 

long int  startZero;
int backupMode;

struct vehicle
{
	double fa; //final accln 
	double sa; //initial accln
	double sv; //initial velocity 
	double ca; //current accln 
	double cv; //currect velocity
	double subtime; //time diff per update 
	double stime; // start time 
	uint8_t d_cnt;
	double d_i[3]; // for storing triples of distance between two cars  
	double distn;
	uint8_t r_cnt;
	double r_i; // store last relative velocity value 
	double alead_d;
	double alead_v; 
	unsigned int stopSend;
	double lossStart;
	double lossDuration;
	int ACCorCACC;
	int CurrentMode;
	int backupMode;
	float probExitGood;
	float probExitBad;
	int LossState;
	double xlast; // for last x position of ego vehicle 
	double xleadlast; // stores last x position of lead vehicle 
};

struct cacc
{
	double error;
	double h;
	double Kp;
	double Kd;
	long int time;


};

class kalman 
{
	public:
	//  this is 1D  kalman filter techinique 
	kalman();
	int dim;  //#of states 
	double A,B,H,P,Q,R,x,xhat;
	double runkalman(double control, double z); //runs kalman filter techniques and returns state variable 
	void setupKalman(double a, double b, double h, double X,  double p, double q, double r);

};


kalman::kalman()
{	
}

double kalman::runkalman(double control, double y)
{
	double pred_state, pred_prob,innov,innov_cov,gain;

	pred_state = this->A *this->x + this->B * control;
	pred_prob = this->A * this->P * this->A + this->Q;
    innov = y - this->H * pred_state; 
	innov_cov = this->H * pred_prob * this->H + this->R;
	gain = pred_prob * this->H * 1.0/ innov_cov; 
	this->x = pred_state + gain *innov; 
	this->P = (1-gain*this->H)*pred_prob;

	return this->x;
}

void kalman::setupKalman(double a, double b, double h, double X, double p, double q, double r)
{
	this->A=a;
	this->B=b;
	this->H=h;
	this->x=X;
	this->P=p;
	this->Q=q;
	this->R=r;
}




class payload
{
public:

	payload(unsigned int length){
		len = length;
		buffer = (uint8_t *) calloc(len,sizeof(uint8_t));
		node=(unsigned int*)calloc(1,sizeof(unsigned int));
		vel =(double*) calloc(1,sizeof(double));
		accln=(double*) calloc(1,sizeof(double));
		x=(double*) calloc(1,sizeof(double));
		y=(double*) calloc(1,sizeof(double));

		node = (unsigned int *)&buffer[0];
		vel = (double *) &buffer[sizeof(unsigned int)];
		accln = (double *) &buffer[sizeof(unsigned int)+sizeof(double)];
		x = (double *) &buffer[sizeof(unsigned int)+sizeof(double)+sizeof(double)];
		y = (double *) &buffer[sizeof(unsigned int)+sizeof(double)+sizeof(double)+sizeof(double)];
		test = (unsigned int*)&buffer[sizeof(unsigned int)+sizeof(double)+sizeof(double)+sizeof(double)+sizeof(double)];
	}



	void setBuffer(unsigned char *B)
	{
		node = (unsigned int *)&B[0];
		vel = (double *) &B[sizeof(unsigned int)];
		accln = (double *) &B[sizeof(unsigned int)+sizeof(double)];
		x = (double *) &B[sizeof(unsigned int)+sizeof(double)+sizeof(double)];
		y = (double *) &B[sizeof(unsigned int)+sizeof(double)+sizeof(double)+sizeof(double)];
		test = (unsigned int*)&buffer[sizeof(unsigned int)+sizeof(double)+sizeof(double)+sizeof(double)+sizeof(double)];

	}



	unsigned int *node;
	double *vel;
	double *accln;
	double *x;
	double *y;
	uint8_t *buffer;
	unsigned int len;
	unsigned int *test;

};

void ReceivePacket(Ptr<Socket> socket);


class VehicleApp
{
public:
	VehicleApp();
	~VehicleApp();
	//void ReceivePacket(Ptr<Socket> socket);
	void GenerateTraffic ();
	//void ScheduleSimulation();

	Ptr<Socket> recvSocket;
	Ptr<Socket> sendSocket;

	unsigned int packetCount;
	unsigned int seqNum;
	double interPacketInterval;
	unsigned int packetSize;
	vehicle cur; //current car info
	vehicle lead; //lead car info 
	kalman kd,kdd; //for distance
	kalman kv; //for velocity 
	kalman ka; // for acceleration 

	unsigned int vehitr;
	unsigned int caccitr;
	double  dmeTime;
	unsigned int dmeitr; 
	double caccint;
	double vehint;
	unsigned int n;
	FILE *mobilityFile;
	FILE *commFile;
	FILE *caccFile;
	FILE *temp;
	cacc C;
	Ptr<UniformRandomVariable> rndvar; 
};

VehicleApp::VehicleApp()
{
	mobilityFile=(FILE *) malloc(sizeof(FILE));
	commFile=(FILE *) malloc(sizeof(FILE));
	caccFile=(FILE *) malloc(sizeof(FILE));


	this->kd.setupKalman(1,0,1,0,1,0.01,3);
	this->kdd.setupKalman(1,0,1,0,1,0.01,3);
	this->kv.setupKalman(-1,1,1,0,1,0.01,3);
	this->ka.setupKalman(1,0,1,0,0.1,0.001,2);


}



VehicleApp::~VehicleApp()
{
	free(mobilityFile);
	free(commFile);
	free(caccFile);

}


void VehicleApp::GenerateTraffic()
{
	     	

		uint32_t index = this->sendSocket->GetNode()->GetId(); //current node id
		payload buf(packetSize);
	    unsigned int i,*t;
	    double vel,acc,x1,y1;

	    packetTag pt;

		Vector mob = this->sendSocket->GetNode()->GetObject<MobilityModel>()->GetPosition(); //current position
	    double velocity = this->cur.cv;

	    if (USE_UT)
				acc = this->cur.fa;
	    else
				acc = this->cur.ca;

	     Ptr<Packet> p = Create<Packet> (packetSize);
	     SeqTsHeader seqTs;
	     seqTs.SetSeq (this->seqNum);
		 p->AddHeader (seqTs);
		 pt.Set(index);
		 p->AddPacketTag(pt);
		 this->sendSocket->Send (p);
	     	
}


class VehicleNodes
{
public:
	VehicleNodes();
	VehicleNodes(unsigned int N1);
	~VehicleNodes();

	void setupNode(unsigned int N1,char *pm);
	void setNodeLocation(unsigned int nodeID,float x,float y,float  z);
	int vehicleConfig(float st, float cu, float vu,float du,float ls, float ld,int a,int bk,int b, int c);
	int packetConfig(uint32_t packetSize1,float numberOfPacket1, double TargetThroughput1);
	int addIPAddress( Ipv4AddressHelper *ip);
	int createSendSocket(unsigned int nodeID, int);
	int createRecvSocket(unsigned int nodeID, int);
	NodeContainer getNode();
	VehicleApp *apps;

	unsigned int N;
	NodeContainer node;
	NetDeviceContainer device;
	Ipv4InterfaceContainer ipAddr;
	MobilityHelper mobility;
	double throughput;
	unsigned int numberOfPacket;
	unsigned int packetSize;
	unsigned int seqNum;
	unsigned int counter;
	unsigned int packetCount;
	Time interPacketInterval;

};



NodeContainer VehicleNodes::getNode()
{
	return node;
}



VehicleNodes::VehicleNodes()
{


}

VehicleNodes::VehicleNodes(unsigned int N1)
{
	N=N1;

}

VehicleNodes::~VehicleNodes()
{
//	fclose(fp);

}




int VehicleNodes::addIPAddress(Ipv4AddressHelper *ip)
{
	Ipv4InterfaceContainer i = ip->Assign (device);

	return 0;
}

int VehicleNodes::createSendSocket(unsigned int nodeID, int port)
{
	  TypeId tid = TypeId::LookupByName ("ns3::UdpSocketFactory");
	  apps[nodeID].sendSocket = Socket::CreateSocket (node.Get (nodeID), tid);
	  InetSocketAddress remote = InetSocketAddress (Ipv4Address ("255.255.255.255"),port);
	  apps[nodeID].sendSocket->SetAllowBroadcast (true);
	  apps[nodeID].sendSocket->Connect (remote);

	  return 0;
}

int VehicleNodes::createRecvSocket(unsigned int nodeID, int port)
{
	  TypeId tid = TypeId::LookupByName ("ns3::UdpSocketFactory");
	  apps[nodeID].recvSocket = Socket::CreateSocket (node.Get (nodeID), tid);
	  InetSocketAddress local = InetSocketAddress (Ipv4Address::GetAny (), port);
	  apps[nodeID].recvSocket->Bind (local);
	  apps[nodeID].recvSocket->SetRecvCallback (MakeCallback(&(ReceivePacket)));

	  return 0;
}

int VehicleNodes::packetConfig(uint32_t packetSize1,float simtime, double TargetThroughput1)
{
	packetSize=packetSize1;
	throughput = TargetThroughput1;
	numberOfPacket=throughput * simtime /(packetSize*8);
	counter=0;
	double interval = packetSize*8.0/(throughput*1.0); // seconds
	interPacketInterval = Seconds (interval);
	seqNum=0;
	packetCount=numberOfPacket;

	for(int i=0;i<N;i++)
	{	apps[i].seqNum=0;
		apps[i].interPacketInterval=interval;
		apps[i].packetCount=packetCount;
		apps[i].packetSize=packetSize;

		apps[i].n = i;
	}
	return 0;

}

int VehicleNodes::vehicleConfig(float st, float cu, float vu, float du,float ls,float ld,int AC,int BK,int gl,int bl)
{
	for(int i=0;i<N;i++)
	{
		apps[i].vehint=vu;
		apps[i].caccint=cu;
		apps[i].dmeTime=du;
		apps[i].vehitr = (unsigned int) (st / vu); 
		apps[i].caccitr = (unsigned int) (st / cu); 
		apps[i].dmeitr=(unsigned int) (st/du);
		apps[i].cur.ca=0;
		apps[i].cur.cv=0;
		apps[i].cur.sa=0;
		apps[i].cur.fa=0;
		apps[i].cur.sv=0;
		apps[i].cur.stime= Now().GetMicroSeconds();
		apps[i].cur.subtime = cu; // time between each cacc updates 
       	apps[i].lead.cv = 0;
   		apps[i].lead.ca = 0;
		apps[i].C.error=0;
		apps[i].cur.d_i[0]=dref/m2ft;
		apps[i].cur.d_i[1]=dref/m2ft;
		apps[i].cur.d_i[2]=dref/m2ft;
		apps[i].cur.d_cnt=0;
		apps[i].cur.r_i=0.0;
		apps[i].cur.stopSend=0;
		apps[i].cur.lossStart=ls;
		apps[i].cur.lossDuration=ld;
		apps[i].cur.ACCorCACC=AC;
		apps[i].cur.CurrentMode=AC;
		apps[i].cur.backupMode=BK;
		apps[i].cur.probExitBad = 1.0/bl;
		apps[i].cur.probExitGood = 1.0/gl;
		apps[i].cur.LossState = 0; // 0 =  no loss state 
		apps[i].rndvar = CreateObject<UniformRandomVariable> ();
		apps[i].rndvar->SetAttribute ("Min", DoubleValue (0.0));
		apps[i].rndvar->SetAttribute ("Max", DoubleValue (1.0));
	}	
}

void VehicleNodes::setNodeLocation( unsigned int nodeID, float x, float y,float z)
{

	Ptr <Node> n = this->node.Get(nodeID);

	MobilityHelper m;
	Ptr<ListPositionAllocator> positionAlloc = CreateObject<ListPositionAllocator> ();
   	positionAlloc->Add (Vector (x,y,z));
   	m.SetPositionAllocator (positionAlloc);
   	m.Install (n);

	this->apps[nodeID].cur.xlast = x;  //x posn of the subject car 
	if (nodeID>0)
	this->apps[nodeID].cur.xleadlast = this->apps[nodeID-1].cur.xlast; // x posn of lead car  
}




void VehicleNodes::setupNode(unsigned int N1,char *pm) //create 1 node
{


	N=N1;
	std::string phyMode (pm);
	node.Create(N);

	apps=new VehicleApp[N];
	// The below set of helpers will help us to put together the wifi NICs we want

	YansWifiChannelHelper waveChannel = YansWifiChannelHelper (); // create channel without any config
	waveChannel.SetPropagationDelay("ns3::ConstantSpeedPropagationDelayModel");

    waveChannel.AddPropagationLoss("ns3::ThreeLogDistancePropagationLossModel","Distance0",DoubleValue(1.0),"Distance1",DoubleValue(210),"Distance2",DoubleValue(286),"Exponent0",DoubleValue(1.9),"Exponent1",DoubleValue(15),"Exponent2",DoubleValue(3.65));
    waveChannel.AddPropagationLoss("ns3::NakagamiPropagationLossModel","Distance1",DoubleValue(80.0),"Distance2",DoubleValue(320),"m0",DoubleValue(1.5),"m1",DoubleValue(0.75),"m2",DoubleValue(0));



	YansWifiPhyHelper wifiPhy =  YansWifiPhyHelper::Default ();
	Ptr<YansWifiChannel> channel = waveChannel.Create ();
	wifiPhy.SetChannel (channel);
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
	//wifiPhy.EnablePcap ("perftest80211p", device);


	mobility.SetPositionAllocator("ns3::RandomRectanglePositionAllocator","X",StringValue("ns3::UniformRandomVariable[Min=0.0|Max=200.0]"),"Y",StringValue("ns3::UniformRandomVariable[Min=0.0|Max=50.0]"));
		mobility.SetMobilityModel ("ns3::ConstantPositionMobilityModel");
	mobility.Install (node);

	InternetStackHelper internet;
	internet.Install (node);

}

VehicleNodes TotalNodes; //global set of nodes


void ReceivePacket(Ptr<Socket> socket)
{	

	  uint32_t curindex = socket->GetNode()->GetId();
	  unsigned int index;
	  packetTag  pt;

	  SnrTag tag;
	  Ptr<Packet> pkt;
	  double temp_snr=9999;

	  pkt = socket->Recv();
	  pkt->PeekPacketTag(pt);

		if(TotalNodes.apps[curindex].cur.LossState == 0) // state  = 0 , is loss free state 
		{

	  		if(pt.Get() == curindex-1){ //only update if from the car ahead
		  		SeqTsHeader seqTs;
		  		pkt->PeekHeader (seqTs);
		  		if (pkt->PeekPacketTag(tag)){
					temp_snr = tag.Get();
		  		}
		  		fprintf(TotalNodes.apps[curindex].commFile,"%lld,%lld,%d,%d,%f\n",Now().GetMicroSeconds(),seqTs.GetTs ().GetMicroSeconds (),seqTs.GetSeq(),pkt->GetSize(),temp_snr);
		  		numberReceived++;
		  		totalLatency+=Now().GetMicroSeconds()-seqTs.GetTs().GetMicroSeconds();
		  		totalBytes+=pkt->GetSize();

		  		TotalNodes.apps[curindex].lead.cv = TotalNodes.apps[curindex-1].cur.cv;

		        if( TotalNodes.apps[curindex].cur.ACCorCACC==CACC_A) //updates with a 
		  			TotalNodes.apps[curindex].lead.ca = TotalNodes.apps[curindex-1].cur.ca; //assuming lead car sends a(t)
		  		if( TotalNodes.apps[curindex].cur.ACCorCACC==CACC_U)
					TotalNodes.apps[curindex].lead.ca = TotalNodes.apps[curindex-1].cur.fa; //assuming lead car sends u(t+tau)
			}
			if(TotalNodes.apps[curindex].cur.stopSend==1)
			{	double rv = TotalNodes.apps[curindex].rndvar->GetValue();
				if(rv <= TotalNodes.apps[curindex].cur.probExitGood) TotalNodes.apps[curindex].cur.LossState =1;
			} 
		}
		else if(TotalNodes.apps[curindex].cur.LossState==1)
		{	if(TotalNodes.apps[curindex].cur.stopSend==1)
			{	double rv = TotalNodes.apps[curindex].rndvar->GetValue();
				if(rv <= TotalNodes.apps[curindex].cur.probExitBad) TotalNodes.apps[curindex].cur.LossState = 0; 
			} 	
			else TotalNodes.apps[curindex].cur.LossState = 0; 
	  	}
}

void ScheduleSimulation(VehicleNodes *p,unsigned int nodeID)
{


	if (p->apps[nodeID].packetCount > 0)
		    {
		      p->apps[nodeID].GenerateTraffic();
		      p->apps[nodeID].seqNum++;
		      p->apps[nodeID].packetCount--;
			 Simulator::Schedule (Seconds(p->apps[nodeID].interPacketInterval),&ScheduleSimulation,p,nodeID);
		    }
		else
		    {
		      p->apps[nodeID].sendSocket->Close ();
		    }
}

void unblockSend(unsigned int index)
{
	TotalNodes.apps[index].cur.stopSend=0;
	cout<<"index =  "<<index<<" Stop send =0 \t"<<Now().GetMicroSeconds()<<endl;
}

void blockSend(unsigned int index)
{	
	TotalNodes.apps[index].cur.stopSend=1; 
        cout<<"index =  "<<index<<"   Stopsend = 1\t"<< Now().GetMicroSeconds()<<endl;	
	Simulator::Schedule (Seconds(TotalNodes.apps[index].cur.lossDuration),&unblockSend,index);
}


void getDME(unsigned int index) // edited to test kalman 
{
   // assuming DME equipment measures distance and acceleration of the car infront 
 if(TotalNodes.apps[index].dmeitr){
   double gaussD=gaussian();
   double gaussR=gaussian();
   
   gaussD *= distnNoise;
   gaussR *= velNoise;
   double distn1 = TotalNodes.node.Get(index)->GetObject<MobilityModel>()->GetDistanceFrom(TotalNodes.node.Get(index-1)->GetObject<MobilityModel>());
   double distn = distn1+gaussD;

    double sn=distn-TotalNodes.apps[index].cur.d_i[0]; //change in distance between two cars 
	double xn = TotalNodes.getNode().Get(index)->GetObject<MobilityModel>()->GetPosition().x - TotalNodes.apps[index].cur.xlast; 

	double dn_1 = TotalNodes.apps[index].kd.runkalman(0.0,sn+xn); //estimates distance travelled by lead vehicle 
	TotalNodes.apps[index].cur.xlast = TotalNodes.getNode().Get(index)->GetObject<MobilityModel>()->GetPosition().x; 

	double posnlead = TotalNodes.getNode().Get(index-1)->GetObject<MobilityModel>()->GetPosition().x;



	TotalNodes.apps[index].cur.xleadlast=posnlead;

	double relvel =TotalNodes.apps[index-1].cur.cv-TotalNodes.apps[index].cur.cv+gaussR;
	double vn = TotalNodes.apps[index].cur.cv;

	double uv = 2.0*dn_1/TotalNodes.apps[index].dmeTime; 


	double vn_1 = TotalNodes.apps[index].kv.runkalman(uv,relvel+vn);
 
 
	fprintf(TotalNodes.apps[index].temp,"%f\n",vn_1);


	TotalNodes.apps[index].cur.alead_v = (vn_1-TotalNodes.apps[index].cur.r_i)/TotalNodes.apps[index].dmeTime;

	TotalNodes.apps[index].cur.r_i=vn_1; 
	TotalNodes.apps[index].cur.d_i[0]=distn;

	TotalNodes.apps[index].dmeitr--;

	Simulator::Schedule (Seconds(TotalNodes.apps[index].dmeTime),&getDME,index);
 }

}



void followNode(unsigned int index)
{

	if(TotalNodes.apps[index].caccitr){

	double distg = gaussian() * distnNoise; 

	double distnc = TotalNodes.node.Get(index)->GetObject<MobilityModel>()->GetDistanceFrom(TotalNodes.node.Get(index-1)->GetObject<MobilityModel>());
	double distn1=distnc+distg;
	double distn = TotalNodes.apps[index].kdd.runkalman(0.0,(distn1)); 


	double dri = dref*(1.0)/m2ft + h * TotalNodes.apps[index].cur.cv;
	
	double timenow = Now().GetMicroSeconds();
	
	double deltaT=TotalNodes.apps[index].caccint;
	double error = distn-dri;
	double derror = (error-TotalNodes.apps[index].C.error)/deltaT; //rate of error change 



	double alead=TotalNodes.apps[index].lead.ca; 
	double acur=TotalNodes.apps[index].cur.ca;

	if (TotalNodes.apps[index].cur.LossState==1) // invalid communication 
		 TotalNodes.apps[index].cur.ACCorCACC=TotalNodes.apps[index].cur.backupMode;
	else if (TotalNodes.apps[index].cur.LossState==0) 
		 TotalNodes.apps[index].cur.ACCorCACC=TotalNodes.apps[index].cur.CurrentMode; //// this is stupidely wrong. 
	
	
	if( TotalNodes.apps[index].cur.ACCorCACC==ACC) alead=TotalNodes.apps[index].lead.ca; // if ACC, the alead term is set to last available lead acceleration
	else if ( TotalNodes.apps[index].cur.ACCorCACC==CACC_DU) 
		alead  = acur + TotalNodes.apps[index].cur.alead_d; //estimated value is approach acceleration 
	else if ( TotalNodes.apps[index].cur.ACCorCACC==CACC_RU)
		alead = TotalNodes.apps[index].cur.alead_v;  // estimated value is the acceleration 
	else  
		alead = TotalNodes.apps[index].lead.ca;
 
	double newacc = deltaT*((Kp*error+Kd*derror)/h+alead/h-acur/h)+acur;


	
	
	// limit acceleration 
	if (newacc < -maxacc) newacc=-maxacc;
	
	if(newacc > maxacc) newacc = maxacc;
	
	fprintf(TotalNodes.apps[index].caccFile,"%-8.6f %-8.6f %-8.4f %-8.4f %-8.2f %-8.2f %-8.2f %-8.2f %-8.2f %-8.2f\n",timenow,deltaT,error*m2ft,derror*m2ft,dri*m2ft,distnc*m2ft,distn1*m2ft,distn*m2ft,alead*m2ft,newacc*m2ft);


	//update the node
	TotalNodes.apps[index].cur.fa = newacc;
	TotalNodes.apps[index].cur.sa = acur;
	TotalNodes.apps[index].cur.sv=TotalNodes.apps[index].cur.cv; 
	TotalNodes.apps[index].cur.stime=Now().GetMicroSeconds();

	TotalNodes.apps[index].C.error = error;
	TotalNodes.apps[index].C.time = Now().GetMicroSeconds();

	TotalNodes.apps[index].caccitr--; 
	Simulator::Schedule (Seconds(TotalNodes.apps[index].caccint),&followNode,index);
	}


}

void moveNode(unsigned int index)
{

	double distn1,timenow,error;	
	if(TotalNodes.apps[index].vehitr){

		double nowtime=Simulator::Now().GetMicroSeconds();
		double dtime=TotalNodes.apps[index].vehint;
		double slope=(TotalNodes.apps[index].cur.fa-TotalNodes.apps[index].cur.sa)/TotalNodes.apps[index].cur.subtime; //acceleration slope 
		double d=TotalNodes.apps[index].cur.cv*dtime+TotalNodes.apps[index].cur.ca*dtime*dtime/2.0+slope*dtime*dtime*dtime/6.0;
		double vel=TotalNodes.apps[index].cur.cv+TotalNodes.apps[index].cur.ca*dtime+slope*dtime*dtime/2.0;	
		double accln=TotalNodes.apps[index].cur.ca+slope*dtime;

		Ptr <MobilityModel> pos = TotalNodes.getNode().Get(index)->GetObject<MobilityModel>(); 
                Vector posv = pos->GetPosition();
		double x=pos->GetPosition().x;
                pos->SetPosition(Vector(x+d,0,0));
		TotalNodes.apps[index].cur.ca=accln;
		TotalNodes.apps[index].cur.cv=vel;
		if(index>0)
		fprintf(TotalNodes.apps[index].mobilityFile,"%-4.4f %-10.1f %-8.4f %-8.4f %-8.2f %8.2f %-8.2f %-8.2f %-8.2f\n",x*m2ft,nowtime,TotalNodes.apps[index].cur.subtime,dtime,TotalNodes.apps[index].cur.sa*m2ft,TotalNodes.apps[index].cur.fa*m2ft,TotalNodes.apps[index].cur.cv*m2ft,TotalNodes.apps[index-1].cur.cv*m2ft,TotalNodes.apps[index].cur.ca*m2ft);
		else
		fprintf(TotalNodes.apps[index].mobilityFile,"%-4.4f %-10.1f %-8.4f %-8.4f %-8.2f %-8.2f %-8.2f %-8.2f\n",x*m2ft,nowtime,TotalNodes.apps[index].cur.subtime,dtime,TotalNodes.apps[index].cur.sa*m2ft,TotalNodes.apps[index].cur.fa*m2ft,TotalNodes.apps[index].cur.cv*m2ft,TotalNodes.apps[index].cur.ca*m2ft);		
		TotalNodes.apps[index].vehitr--;
		Simulator::Schedule (Seconds(TotalNodes.apps[index].vehint),&moveNode,index);

	}
}


void leadNodeFile(unsigned int index,double nodeupdate,unsigned  int count,  FILE *fp)
{ 
		double acc;
		char sget[100];

		if(fscanf(fp,"%s",sget)!=EOF and --count>=0 ){
			acc = (double) atof(sget)/m2ft; // acc is converted to m/s^2

			TotalNodes.apps[index].cur.fa = acc;
			TotalNodes.apps[index].cur.sa = TotalNodes.apps[index].cur.ca;
			TotalNodes.apps[index].cur.sv =TotalNodes.apps[index].cur.cv;
			TotalNodes.apps[index].cur.stime=Now().GetMicroSeconds();

			Simulator::Schedule (Seconds(nodeupdate),&leadNodeFile,index,nodeupdate,count,fp);
		}
}






int main(int argc, char *argv[])
{

	uint32_t SEED=100;
	uint64_t RUN;
	unsigned int N =2;
	int i=0;
	unsigned int PktSize=200;
	double thrpt = 64000;

	unsigned int ChainLength;
	float maxx,maxy;
	char physicalMode[50];
	char logloc[50];
	FILE *fp;
	double simtime,caccupdate,moveupdate,leadupdate,dmeupdate; 
	double lossStart,lossDuration; 
	unsigned int lossNum;
	int burstlength,goodlength; 
	int ACCorCACC;

	lossRate=0;
	totalLatency=0;
	totalBytes=0;
	totalTime=0;
	numberReceived=0;

	std::string animFile = "animationgrid.xml";
	unsigned int iteration=1000;

	if (argc != 25)
	{ printf("Error argc = %d \n Usage : caccsim <RANDSEED> <numberofNodes> <datarate(kbps)> <packetsize(bytes)> <SimTime sec> <phymode> <chainlength> <headwaytime> <accln profile> <caccupdatetime> <movetime> <leadupdate> <Dref>  <ACC_basic(0)orCACC_U(1)orLOCAL_DU(2)orCACC_A(3)orLOCAL_RU(4) <DMEupdatetime> <logLocation> <LossStartTime> <LossDuration> <numLossVehicles> <BurstLength> <GoodLength> <BackupMode> <distnNoise std> <velNoise stf> \n",argc);
		exit (1);
	}

	else
	{
		RUN=atoi(argv[1]);
		N = atoi(argv[2]);
		thrpt = atoi(argv[3])*1000.0;
		PktSize = atoi(argv[4]);
		simtime = atof(argv[5]); //total sim time
		strcpy(physicalMode,argv[6]);
		ChainLength = atoi(argv[7]);
		h=atof(argv[8]);
		fp = fopen(argv[9],"r"); 
		caccupdate = atof(argv[10]); //time between each cacc update
		moveupdate = atof(argv[11]); //time beteween each veh move 
		leadupdate = atof(argv[12]);
		dref=atof(argv[13]);
		ACCorCACC=atoi(argv[14]);
		dmeupdate=atof(argv[15]);
		strcpy(logloc,argv[16]);
		lossStart=atof(argv[17]);
		lossDuration=atof(argv[18]);
		lossNum=atoi(argv[19]);	
		burstlength = atoi(argv[20]);
		goodlength = atoi(argv[21]);
		backupMode = atoi(argv[22]);
		distnNoise = atof(argv[23]);
		velNoise = atof(argv[24]);
	}

	printf(" Setup:: ACCorCACC = %d , simtime=%f, throughput= %f, chainlength=%d, h=%f, dref=%f, caccupdate=%f, moveupdate=%f, leadupdate=%f dmeupdate=%f, loglocation=%s, LossStart=%f, LossDuration=%f, LossNum=%d, burstlength=%d, goodlength=%d, probexitbad=%f, probexitgood=%f, fallback=%d\n",ACCorCACC,simtime,thrpt,ChainLength,h,dref,caccupdate,moveupdate,leadupdate,dmeupdate,logloc,lossStart,lossDuration,lossNum,burstlength,goodlength,1.0/burstlength,1.0/goodlength, backupMode); 
	printf(" distnNoise  =%f, Vel Noise = %f\n",distnNoise,velNoise);
	printf("vehitr = %d, caccitr = %d, leaditr = %d, broadcastcount=%d, dmecount=%d\n", (int) (simtime/moveupdate), (int) (simtime/caccupdate), (int) (simtime/leadupdate),(int)(simtime*thrpt/(PktSize*8)),(int) (simtime/dmeupdate));	

	RngSeedManager::SetSeed (RUN);
	RngSeedManager::SetRun (RUN);

   	TotalNodes.setupNode(N,physicalMode);
   		char  nameM[100];
   		char nameE[100];
		char nameC[100];
		char nameX[100];

   	// create chain of vehicles
   	cout << "chainlength ==   "<<ChainLength<<endl;
   	for(i=0;i<ChainLength;i++){
   		TotalNodes.setNodeLocation(i,(ChainLength-i-1)*dref/m2ft,0,0);

   		sprintf(nameM,"%s/%s_%d.dat",logloc,"mobilityFile",i);
   		sprintf(nameE,"%s/%s_%d.dat",logloc,"commFile",i);
		sprintf(nameC,"%s/%s_%d.dat",logloc,"caccFile",i);
		sprintf(nameX,"%s/%s_%d.dat",logloc,"VarFile",i);
   		cout<<nameM<<"\t"<<nameE<<"\t"<<nameC<<nameX<<endl;
		TotalNodes.apps[i].mobilityFile = fopen(nameM,"w");
   		TotalNodes.apps[i].commFile = fopen(nameE,"w");
		TotalNodes.apps[i].caccFile = fopen(nameC,"w");
		TotalNodes.apps[i].temp = fopen(nameX,"w");
   	}

	TotalNodes.packetConfig(PktSize,simtime,thrpt);
	TotalNodes.vehicleConfig(simtime,caccupdate,moveupdate,dmeupdate,lossStart,lossDuration,ACCorCACC,backupMode,goodlength,burstlength);


	Ipv4AddressHelper ipv4;
	ipv4.SetBase ("10.2.1.0", "255.255.255.0");
	TotalNodes.ipAddr = ipv4.Assign(TotalNodes.device); 

	for(i=0;i<ChainLength;i++){

		if(i>0){
			TotalNodes.createRecvSocket(i,81); //following car listen and send
			TotalNodes.createSendSocket(i,81);
		}
		else // first car sends only
			TotalNodes.createSendSocket(i,81);



	}

	for(i=ChainLength;i<N;i++) //rest cars send
	{
		TotalNodes.createSendSocket(i,80);
	}

	cout<<"ready to schedule\n";
	for(i=0;i<N;i++)
	{
		if(TotalNodes.apps[i].cur.ACCorCACC==CACC_U or TotalNodes.apps[i].cur.ACCorCACC==CACC_A) // only broadcast if CACC newu or new a
			Simulator::Schedule(Seconds(0.01*(i*0.01)),&ScheduleSimulation,&TotalNodes,i); //al nodes transmit
		
		Simulator::Schedule(Seconds(0.1*(i*0.01)),&moveNode,i);
		
		if(i==0)
			Simulator::Schedule(Seconds(0.1*(i*0.01)),&leadNodeFile,i,leadupdate,(unsigned int) (simtime/leadupdate),fp);
		else if(i>0 && i<ChainLength){
			Simulator::Schedule(Seconds(0.01*(i*0.01)),&followNode,i);
			Simulator::Schedule(Seconds(0.01*(i*0.01)),&getDME,i);
		}
		if(i>0 and i<=lossNum) // number of cars behind car 0 under loss  
		{
			Simulator::Schedule(Seconds(TotalNodes.apps[i].cur.lossStart),blockSend,i);	
		}
	}

	totalBytes=0;
	totalTime=0;

	Simulator::Run ();
	Simulator::Destroy();
	return 0;
}
