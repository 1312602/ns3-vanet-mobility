/* -*-  Mode: C++; c-file-style: "gnu"; indent-tabs-mode:nil; -*- */


/*
	Single vehicle on each lane, radio range test
*/

#include <fstream>
#include <iostream>
#include <iomanip>
#include "ns3/core-module.h"
#include "ns3/common-module.h"
#include "ns3/node-module.h"
#include "ns3/helper-module.h"
#include "ns3/mobility-module.h"
#include "ns3/contrib-module.h"
#include "ns3/wifi-module.h"
#include "ns3/random-variable.h"
#include "math.h"
#include "Highway.h"
#include <list>

NS_LOG_COMPONENT_DEFINE ("HADI");

using namespace ns3;
using namespace std;

static Ptr<Vehicle> CreateVehicle(Ptr<Highway> highway, int lane, int direction);
static Ptr<Vehicle> CreateRSU(Ptr<Highway> highway, int position, int lane, bool silent);
static void ReceiveData(Ptr<Vehicle> veh, Ptr<const Packet> packet, Address address);
static void ReBroadcastMessage(Ptr<Vehicle> vehicle, unsigned int pID);

static void Start(Ptr<Highway> highway)
{
  highway->Start();
}

static void Stop(Ptr<Highway> highway)
{
  highway->Stop();
}

static Ptr<Vehicle> CreateVehicle(Ptr<Highway> highway, int lane, int direction)
{
	Ptr<Vehicle> vhc=CreateObject<Vehicle>();

	vhc->SetupWifi(highway->GetWifiHelper(), highway->GetYansWifiPhyHelper(), highway->GetNqosWifiMacHelper());
	vhc->SetReceiveCallback(highway->GetReceiveDataCallback());

	int VID=highway->GetLastVehicleId();
	VID++;
	highway->SetGlobalVehicleId(VID);
	vhc->SetVehicleId(VID);

	if(direction==1)
		vhc->SetPosition(Vector(-4,highway->GetYForLane(0,1),0));
	else if(direction==-1)
		vhc->SetPosition(Vector(highway->GetHighwayLength()+4,highway->GetYForLane(1,-1),0));

	vhc->SetDirection(direction);
	vhc->SetLane(lane);
	vhc->SetVelocity(30.0);
	vhc->SetAcceleration(0.0);
	Ptr<Model> vhcModel=highway->CreateSedanModel();
	vhcModel->SetDesiredVelocity(30.0);  // max speed 36(m/s)
	vhc->SetModel(vhcModel);          // or common sedan model: highway->GetSedanModel()
	vhc->SetLength(4);
	vhc->SetWidth(2);

	return vhc;
}

static Ptr<Vehicle> CreateRSU(Ptr<Highway> highway, int position, int lane, bool silent)
{
	Ptr<Vehicle> rsu=CreateObject<Vehicle>();

	rsu->SetupWifi(highway->GetWifiHelper(), highway->GetYansWifiPhyHelper(), highway->GetNqosWifiMacHelper());
	rsu->SetReceiveCallback(highway->GetReceiveDataCallback());

	int VID=highway->GetLastVehicleId();
	VID++;
	highway->SetGlobalVehicleId(VID);
	rsu->SetVehicleId(VID);

	int direction = 1; 	// doesn't matter for an RSU
	rsu->SetPosition(Vector(position,highway->GetYForLane(lane,1),0));
	rsu->SetDirection(direction);

	rsu->SetLane(lane);
	rsu->SetVelocity(0.0); 		// RSU is stopped
	rsu->SetAcceleration(0.0);
	Ptr<Model> rsuModel=highway->CreateSedanModel();
	rsuModel->SetDesiredVelocity(0.0);
	rsu->SetModel(rsuModel);
	rsu->SetLength(1);
	rsu->SetWidth(1);
	rsu->SetSilence(silent);

	return rsu;
}

static bool InitVehicle(Ptr<Highway> highway, int& VID)
{
	ns3::Time nowtime = ns3::Simulator::Now();

	cout << nowtime.ns3::Time::GetSeconds() <<'\t' << "[H] ";
	cout << "Highway Init" << '\n';

	int RSUdistance=5000;
	double RSUdstPos; RSUdstPos = (highway->GetHighwayLength() - RSUdistance)/2;

	Ptr<Vehicle> RSUdst=CreateRSU(highway, RSUdstPos, true);
	Ptr<Vehicle> RSUsrc=CreateRSU(highway, RSUdstPos+RSUdistance, false);

	// get RSUsrc broadcasting packet 1
	unsigned int pID=1;
	RSUsrc->AddPacket(pID);
	cout << nowtime.ns3::Time::GetSeconds() <<'\t' << "[H] ";
	cout << "TRACE packetID " << pID << " start" << endl;

	highway->AddVehicle(RSUsrc); highway->AddVehicle(RSUdst);

	cout << "\t\tCreated two RSUs at " << RSUdstPos << " and " << RSUdstPos+RSUdistance << endl;

	// TODO initiate expon generation
	// For now just 1 vehicle on opposite lane


	return true;
}

static bool ControlVehicle(Ptr<Highway> highway, Ptr<Vehicle> vehicle, double dt)
{
//	Vector vehiclePos = vehicle->GetPosition();
//	ns3::Time nowtime = ns3::Simulator::Now();
//	cout << nowtime.ns3::Time::GetSeconds() <<'\t' << "[" << vehicle->GetVehicleId() << "] ";
//	cout << "Position " << vehiclePos.x << "\t" << vehiclePos.y << "\t" << vehiclePos.z << endl;

	// Broadcast all packets in broadcast buffer, unless isSilent
	if( vehicle->GetSilence()==false )
	{
		list<unsigned int> plist = vehicle->GetPacketList();	// get vehicle's packet list
		list<unsigned int>::iterator iter1 = plist.begin();				// iterator
		while( iter1 != plist.end() )
		{
//			cout << "\t\t Rebroadcasting pID " << *iter1 << endl;

			Simulator::Schedule(Seconds(0.1),&ReBroadcastMessage, vehicle, *iter1);

			// next
			++iter1;
		}
	}

	return true;
}

static void ReBroadcastMessage(Ptr<Vehicle> vehicle, unsigned int pID)
{
//	ns3::Time nowtime = ns3::Simulator::Now();

	if(vehicle->IsAlive()==true && vehicle->GetSilence()==false)
	{
		stringstream msg;
		msg << pID;

		Ptr<Packet> packet = Create<Packet>((uint8_t*) msg.str().c_str(), msg.str().length());
		vehicle->SendTo(vehicle->GetBroadcastAddress(), packet);
	}
//	else
//	{
//		cout << nowtime.ns3::Time::GetSeconds() <<'\t' << "[" << vehicle->GetVehicleId() << "] ";
//		cout << "DEBUG got a dead vehicle, stopping rebroadcast" << endl;
//	}
}

//static void BroadcastMessage(Ptr<Vehicle> vehicle)
//{
//	ns3::Time nowtime = ns3::Simulator::Now();
//
//	if(vehicle->IsAlive()==true)
//	{
//		stringstream msg;
//		unsigned int pID=1;
//		msg << pID;
//
//		Ptr<Packet> packet = Create<Packet>((uint8_t*) msg.str().c_str(), msg.str().length());
//		vehicle->SendTo(vehicle->GetBroadcastAddress(), packet);
//
//		Simulator::Schedule(Seconds(1.0),&BroadcastMessage, vehicle); // rebroadcast every second
//	}
//	else
//	{
//		cout << nowtime.ns3::Time::GetSeconds() <<'\t' << "[" << vehicle->GetVehicleId() << "] ";
//		cout << "DEBUG got a dead vehicle, stopping rebroadcast" << endl;
//	}
//}

static void ReceiveData(Ptr<Vehicle> veh, Ptr<const Packet> packet, Address address)
{
	ns3::Time nowtime = ns3::Simulator::Now();

	cout << nowtime.ns3::Time::GetSeconds() << '\t' << "[" << veh->GetVehicleId() << "] ";

	// Extract PacketID
	string data=string((char*)packet->PeekData());
	stringstream ss (stringstream::in | stringstream::out);
	unsigned int pID;
	ss << data;
	ss >> pID;

	cout << "Got a message pID " << pID;

	// Check if we have it already
	list<unsigned int> plist = veh->GetPacketList();	// get vehicle's packet list
	list<unsigned int>::iterator iter1 = plist.begin();		// iterator
	bool isNew=true;
	while( iter1 != plist.end() && isNew==true )
	{
		if(*iter1 == pID) isNew=false;
		++iter1;
	}

	// If not, store on rebroadcast buffer, unless silent
	if(isNew==true)
	{
		if(veh->GetSilence()==false)
		{
			// TODO immediate rebroadcast
			veh->AddPacket(pID);
			cout << " and is new, storing" << endl;
		}
		else
		{
			cout << '\n';
			cout << nowtime.ns3::Time::GetSeconds() << '\t' << "[" << veh->GetVehicleId() << "] ";
			cout << "TRACE packetID " << pID << " start" << endl;
		}
	}
	else cout << " not new, discarding" << endl;

//  string data=string((char*)packet->PeekData());
//  stringstream ss (stringstream::in | stringstream::out);
//
//  double obs_id, obs_x;
//  ss << data;
//  ss >> obs_id;
//  ss >> obs_x;
//
//  int vid=veh->GetVehicleId();
//  double now=Simulator::Now().GetSeconds();
//
//  if(!Plot)
//    cout << "at t=" << now << " vehicle " << vid << " received message=[" << data << "]" << endl;

}

int main (int argc, char *argv[])
{ 
	NS_LOG_COMPONENT_DEFINE("2lanes1veh");

	ns3::PacketMetadata::Enable();
	Config::SetDefault ("ns3::WifiRemoteStationManager::FragmentationThreshold", StringValue ("2200"));
	Config::SetDefault ("ns3::WifiRemoteStationManager::RtsCtsThreshold", StringValue ("2200"));
	

	double deltaT=0.1;		// Simulation update rate, could go lower
	RandomVariable RVSpeed;

	// Defaults
	float simTime=1500.0;		//
	int runNumber=1;
	double transmissionPower=21.5;

	Ptr<Highway> highway=CreateObject<Highway>();

	// Setup parameters for highway
	highway->SetHighwayLength(6000);				// 5km
	highway->SetLaneWidth(5);
	highway->SetNumberOfLanes(4);					// TODO lane problem could be here
	highway->SetChangeLane(false);					// No lane change
	highway->SetTwoDirectional(true);				// Two directions
//	highway->SetMedianGap(5);						//

	highway->SetAutoInject(false);					// Manual injection
//	highway->SetSpeedRV(RVSpeed);					// From fixed deterministic speed
//	highway->SetFlowPositiveDirection(flow1);		// Traffic flow in veh/s at entrance
//	highway->SetVelocityPositiveDirection(30);		// Speed at entrance
//	highway->SetFlowNegativeDirection(flow2);
//	highway->SetVelocityNegativeDirection(30);
//	highway->SetFlowRVPositiveDirection(RV1);		// Distribution
//	highway->SetFlowRVNegativeDirection(RV2);
//	highway->SetPenetrationRate(100);				// Penetration rate of equipments
	highway->SetDeltaT(deltaT);						// Mobility step, defined above
  
	// Update the transmission range of wifi shared in the Highway.
	YansWifiPhyHelper tempHelper = highway->GetYansWifiPhyHelper();
	tempHelper.Set("TxPowerStart",DoubleValue(transmissionPower)); // 250-300 meter transmission range TODO reduce power to keep range close to 250
	tempHelper.Set("TxPowerEnd", DoubleValue(transmissionPower));   // 250-300 meter transmission range
	highway->SetYansWifiPhyHelper(tempHelper);
  

	// Try sample function called for every mobility update and every vehicle
	highway->SetControlVehicleCallback(MakeCallback(&ControlVehicle));
	highway->SetInitVehicleCallback(MakeCallback(&InitVehicle));
	highway->SetReceiveDataCallback(MakeCallback(&ReceiveData));

	// Setup seed and run-number (to affect random variable outcome of different runs)
	if(runNumber < 1) runNumber=1;
	SeedManager::SetSeed(1);
	SeedManager::SetRun(runNumber);

	// Logging & Tracing
//	AsciiTraceHelper ascii;
//	Ptr<OutputStreamWrapper> outstream = ascii.CreateFileStream("2lanes1veh.tr");
//	highway->GetYansWifiPhyHelper().EnableAsciiAll(outstream);
//	highway->GetWifiHelper().EnableLogComponents();

	// Schedule and run highway
	Simulator::Schedule(Seconds(0.0), &Start, highway);		// Invokes Start(Highway)
	Simulator::Schedule(Seconds(simTime), &Stop, highway);
	Simulator::Stop(Seconds(simTime));
	Simulator::Run();
	Simulator::Destroy();

	return 0;
}
