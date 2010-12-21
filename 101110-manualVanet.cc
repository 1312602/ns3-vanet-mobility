/* -*-  Mode: C++; c-file-style: "gnu"; indent-tabs-mode:nil; -*- */


/*
	Manual simulation. Manual injection of vehicles. Verify movement, positions, connectivity.
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
//#include "101110-Controller.h"

NS_LOG_COMPONENT_DEFINE ("HADI");

using namespace ns3;
using namespace std;

static void Start(Ptr<Highway> highway)
{
  highway->Start();
}

static void Stop(Ptr<Highway> highway)
{
  highway->Stop();
}

//static void HighwayAddVehicle(Ptr<Highway> highway, Ptr<Vehicle> vehicle)
//{
//	highway->AddVehicle(vehicle);
//}

static Ptr<Vehicle> CreateVehicle(Ptr<Highway> highway)
{
	Ptr<Vehicle> vhc=CreateObject<Vehicle>();
//	vhc->SetupWifi(highway->GetWifiHelper(), policePhyHelper, highway->GetNqosWifiMacHelper());
	{
		int VID=highway->GetLastVehicleId();
		VID++;
		highway->SetGlobalVehicleId(VID);
		vhc->SetVehicleId(VID);
	}
	vhc->SetDirection(1);
	vhc->SetLane(1);
	vhc->SetPosition(Vector(0.0, highway->GetYForLane(1,1), 0));
	vhc->SetVelocity(30.0);
	vhc->SetAcceleration(0.0);
	Ptr<Model> vhcModel=highway->CreateSedanModel();
	vhcModel->SetDesiredVelocity(30.0);  // max speed 36(m/s)
	vhc->SetModel(vhcModel);          // or common sedan model: highway->GetSedanModel()
//	vhc->SetLaneChange(highway->GetSedanLaneChange());
	vhc->SetLength(4);
	vhc->SetWidth(2);
//	vhc->SetReceiveCallback(highway->GetReceiveDataCallback());

	return vhc;
}

static void ExponentialAddVehicles(Ptr<Highway> highway)
{
	ns3::Time nowtime = ns3::Simulator::Now();
	Ptr<Vehicle> vehicle1=CreateVehicle(highway);
    highway->AddVehicle(vehicle1);

    cout << "\tTICK "<< nowtime.ns3::Time::GetSeconds() << " Added vehicle" << endl;

//	Recursive call & schedule
    RandomVariable RV1=highway->GetFlowRVPositiveDirection();
	double deltaExp = RV1.GetValue();
	Simulator::Schedule(Seconds(deltaExp), &ExponentialAddVehicles, highway);		// Schedule ExponentialAddVehicles(Highway)
	cout << "\tScheduled new vehicle for " << nowtime.ns3::Time::GetSeconds() + deltaExp << " Delta "<< deltaExp << endl;
}

static bool InitVehicle(Ptr<Highway> highway, int& VID)
{

	cout << "Highway Init" << '\n';
	cout << "Beginning recursive exponential generation of vehicles..." << '\n';

	double lambdaS=0.0039*30; // veh/m * m/s
	// The next line is gettin an exponential variable with seconds per vehicle, and an upper bound 5 times higher
	RandomVariable RV1 = ExponentialVariable(1/(lambdaS), 5/lambdaS);	// mean, upperbound
	highway->SetFlowRVPositiveDirection(RV1);		// Save distribution with Highway

	ExponentialAddVehicles(highway);

	return true;
}

static bool ControlVehicle(Ptr<Highway> highway, Ptr<Vehicle> vehicle, double dt)
{
	Vector vehiclePos = vehicle->GetPosition();
	ns3::Time nowtime = ns3::Simulator::Now();
//	cout << "TICK " << nowtime.ns3::Time::GetSeconds();
//	cout << " Vehicle " << vehicle->GetVehicleId() << " Position " << vehiclePos.x << " " << vehiclePos.y << " " << vehiclePos.z << endl;

	return true;
}

//static void BroadcastWarning(Ptr<Vehicle> veh)
//{
//  stringstream msg;
//  msg << veh->GetVehicleId()
//      << " " << veh->GetPosition().x
//      << " has blocked the road at x=" << veh->GetPosition().x
//      << " direction=" << veh->GetDirection()
//      << " lane=" << veh->GetLane();
//
//  Ptr<Packet> packet = Create<Packet>((uint8_t*) msg.str().c_str(), msg.str().length());
//
//  veh->SendTo(veh->GetBroadcastAddress(), packet);
//
//  Simulator::Schedule(Seconds(5.0), &this, veh);
//}

int main (int argc, char *argv[])
{ 
	ns3::PacketMetadata::Enable();
	Config::SetDefault ("ns3::WifiRemoteStationManager::FragmentationThreshold", StringValue ("2200"));
	Config::SetDefault ("ns3::WifiRemoteStationManager::RtsCtsThreshold", StringValue ("2200"));
	

	double deltaT=0.1;		// Simulation update rate, could go lower
	RandomVariable RVSpeed;

	// Defaults
	float simTime=175.0;		// Enough for 1 vehicle to cross the 10km of road at 30m/s
	int runNumber=1;
	string directory="./";
	string fp="";
//	int distribution=0;
//	double std1=0.0, std2=0.0;
	double transmissionPower=21.5;


	// Setup Speed (constant, 30m/s)
//	double speedvector[1]={30};
//	RVSpeed = DeterministicVariable (speedvector, 1); 	// 1 value, 30m/s TODO km/h or m/s

	// Setup Flow Rate Distribution TODO Setup exponential vehicle creation
//	RV1 = ExponentialVariable(flow1*deltaT, maxFlow*deltaT);
//	RV2 = ExponentialVariable(flow2*deltaT, maxFlow*deltaT);

	Ptr<Highway> highway=CreateObject<Highway>();
//	Ptr<Controller> controller=CreateObject<Controller>();

	// Bind an experiment (controller) to highway
//	controller->SetHighway(highway);
//	controller->Plot=false;		// no idea of what is plotting vehicles

	// Setup parameters for highway
	highway->SetHighwayLength(5000);				// 5km
	highway->SetLaneWidth(5);
	highway->SetNumberOfLanes(2);					// TODO 2 lanes means 2 or 4?
	highway->SetChangeLane(false);					// No lane change
	highway->SetTwoDirectional(true);				// Two directions
	highway->SetMedianGap(5);						// TODO clarify MedianGap

	highway->SetAutoInject(false);					// Manual injection
//	highway->SetSpeedRV(RVSpeed);					// From fixed deterministic speed
//	highway->SetFlowPositiveDirection(flow1);		// TODO Traffic flow in veh/s at entrance
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
  
	// Thanks, I'll take it from here.
	// Bind the Highway/Vehicle events to the event handlers. Controller's will catch them.
//	highway->SetControlVehicleCallback(MakeCallback(&Controller::ControlVehicle,controller));
//	highway->SetInitVehicleCallback(MakeCallback(&Controller::InitVehicle,controller));
//	highway->SetReceiveDataCallback(MakeCallback(&Controller::ReceiveData,controller));

	// Try sample function called for every mobility update and every vehicle
	highway->SetControlVehicleCallback(MakeCallback(&ControlVehicle));
	highway->SetInitVehicleCallback(MakeCallback(&InitVehicle));

	// Setup seed and run-number (to affect random variable outcome of different runs)
	if(runNumber < 1) runNumber=1;
	SeedManager::SetSeed(1);
	SeedManager::SetRun(runNumber);

	// Output summary of parameters' value	// TODO prepare initial output
//	ofstream simPar;
//	string name=fp;
//	fp+="-Parameters";
//	simPar.open(fp.c_str());
//	simPar << "time = " << simTime << " run number = " << runNumber <<endl
//	<< "number of lanes = " << numberOfLanes << " two directional = " << twoDirectional << " lane change = " << laneChange<< endl
//	<< "flow distribution = " << distribution << endl
//	<< "flow1 = " << flow1 << " vel1 = " << vel1 << endl
//	<< "flow2 = " << flow1 << " vel2 = " << vel1 << endl
//	<< "transmission power = " << transmissionPower << endl
//	<< "injection gap = " << gap << " injection mix = " << mix << " penetration rate = " << pRate << endl
//	<< "speed limit = " << speedLimit << " speed std = " << speedStd << endl
//	<< "file prefix = " << name << endl;
//	simPar.close();
  

//	// Add a vehicle
//	Ptr<Vehicle> vehicle1=CreateObject<Vehicle>();
////	vehicle1->SetupWifi(highway->GetWifiHelper(), policePhyHelper, highway->GetNqosWifiMacHelper());
//	vehicle1->SetVehicleId(1);	// VID++
//	vehicle1->SetDirection(1);
//	vehicle1->SetLane(1);
//	vehicle1->SetPosition(Vector(0.0, highway->GetYForLane(1,1), 0));
//	vehicle1->SetVelocity(30.0);
//	vehicle1->SetAcceleration(0.0);
//	Ptr<Model> vehicle1Model=highway->CreateSedanModel();
//	vehicle1Model->SetDesiredVelocity(30.0);  // max speed 36(m/s)
//	vehicle1->SetModel(vehicle1Model);          // or common sedan model: highway->GetSedanModel()
////	vehicle1->SetLaneChange(highway->GetSedanLaneChange());
//	vehicle1->SetLength(4);
//	vehicle1->SetWidth(2);
////	vehicle1->SetReceiveCallback(highway->GetReceiveDataCallback());
////    highway->AddVehicle(vehicle1);
//
//	Simulator::Schedule(Seconds(2.0), &HighwayAddVehicle, highway, vehicle1);

	// Schedule and run highway
	Simulator::Schedule(Seconds(0.0), &Start, highway);		// Invokes Start(Highway)
	Simulator::Schedule(Seconds(simTime), &Stop, highway);
	Simulator::Stop(Seconds(simTime));
	Simulator::Run();
	Simulator::Destroy();

	return 0;
}
