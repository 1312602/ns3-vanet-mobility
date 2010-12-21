/* -*-  Mode: C++; c-file-style: "gnu"; indent-tabs-mode:nil; -*- */
/*
 * Copyright (c) 2005-2009 Old Dominion University [ARBABI]
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
 * Author: Hadi Arbabi <marbabi@cs.odu.edu>
 */

/*
	This the starting point of the simulation and experiments.
	The main function will parse the input and parameter settings.
	Creates a highway and set the highway parameters. then bind the events (callbacks)
	to the created controller and designed handlers. Sets the highway start and end time,
	and eventually runs the simulation which is basically running a highway with a controller.
	You can add your functions to controller to create various scenarios. 
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
#include "101108-Controller.h"

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

int main (int argc, char *argv[])
{ 
	ns3::PacketMetadata::Enable();
	Config::SetDefault ("ns3::WifiRemoteStationManager::FragmentationThreshold", StringValue ("2200"));
	Config::SetDefault ("ns3::WifiRemoteStationManager::RtsCtsThreshold", StringValue ("2200"));
	

	double deltaT=0.1;		// Simulation update rate, could go lower
	RandomVariable RV1, RV2, RVSpeed;

	// Defaults TODO clean unused
	float simTime=1.0;
//	bool plot=false;
//	bool twoDirectional=false;
//	bool laneChange=true;
	double flow1=0.5, flow2=0.5;
//	double vel1=29, vel2=29;
//	double pRate=100;
//	double mix=80;
//	double gap=5;
//	double speedLimit=29;
//	double speedStd=1;
//	int numberOfLanes=3;
	int runNumber=1;
	string directory="./";
	string fp="";
//	int distribution=0;
//	double std1=0.0, std2=0.0;
	double maxFlow=5.0;
	double transmissionPower=21.5;


	// Setup Speed (constant, 30m/s)
	double speedvector[1]={30};
	RVSpeed = DeterministicVariable (speedvector, 1); 	// 1 value, 30m/s TODO km/h or m/s

	// Setup Flow Rate Distribution TODO Setup exponential vehicle creation
	RV1 = ExponentialVariable(flow1*deltaT, maxFlow*deltaT);
	RV2 = ExponentialVariable(flow2*deltaT, maxFlow*deltaT);

	Ptr<Highway> highway=CreateObject<Highway>();
	Ptr<Controller> controller=CreateObject<Controller>();

	// Bind an experiment (controller) to highway
	controller->SetHighway(highway);
	controller->Plot=false;		// no idea of what is plotting vehicles

	// Setup parameters for highway
	highway->SetHighwayLength(10000);
	highway->SetLaneWidth(5);
	highway->SetNumberOfLanes(2);					// TODO 2 lanes means 2 or 4?
	highway->SetChangeLane(false);					// No lane change
	highway->SetTwoDirectional(true);				// Two directions
	highway->SetMedianGap(5);						// TODO clarify MedianGap
	highway->SetInjectionGap(5);					// 5 meter minimum distance at vehicle injection
	highway->SetInjectionMixValue(100);				// No trucks
	highway->SetAutoInject(true);					// Auto-inject according to ~Exp above
	highway->SetSpeedRV(RVSpeed);					// From fixed deterministic speed
	highway->SetFlowPositiveDirection(flow1);		// TODO Traffic flow in veh/s at entrance
	highway->SetVelocityPositiveDirection(30);		// Speed at entrance
	highway->SetFlowNegativeDirection(flow2);
	highway->SetVelocityNegativeDirection(30);
	highway->SetFlowRVPositiveDirection(RV1);		// Distribution
	highway->SetFlowRVNegativeDirection(RV2);
	highway->SetPenetrationRate(100);				// Penetration rate of equipments
	highway->SetDeltaT(deltaT);						// Mobility step, defined above
  
	// Update the transmission range of wifi shared in the Highway.
	YansWifiPhyHelper tempHelper = highway->GetYansWifiPhyHelper();
	tempHelper.Set("TxPowerStart",DoubleValue(transmissionPower)); // 250-300 meter transmission range TODO reduce power to keep range close to 250
	tempHelper.Set("TxPowerEnd",DoubleValue(transmissionPower));   // 250-300 meter transmission range
	highway->SetYansWifiPhyHelper(tempHelper);
  
  // Bind the Highway/Vehicle events to the event handlers. Controller's will catch them.  
  highway->SetControlVehicleCallback(MakeCallback(&Controller::ControlVehicle,controller));
  highway->SetInitVehicleCallback(MakeCallback(&Controller::InitVehicle,controller));
  highway->SetReceiveDataCallback(MakeCallback(&Controller::ReceiveData,controller));
  
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
  
	// Schedule and run highway
	Simulator::Schedule(Seconds(0.0), &Start, highway);		// Invokes Start(Highway)
	Simulator::Schedule(Seconds(simTime), &Stop, highway);
	Simulator::Stop(Seconds(simTime));
	Simulator::Run();
	Simulator::Destroy();

	return 0;
}
