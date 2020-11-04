/*
 *    Copyright (C) 2020 by YOUR NAME HERE
 *
 *    This file is part of RoboComp
 *
 *    RoboComp is free software: you can redistribute it and/or modify
 *    it under the terms of the GNU General Public License as published by
 *    the Free Software Foundation, either version 3 of the License, or
 *    (at your option) any later version.
 *
 *    RoboComp is distributed in the hope that it will be useful,
 *    but WITHOUT ANY WARRANTY; without even the implied warranty of
 *    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *    GNU General Public License for more details.
 *
 *    You should have received a copy of the GNU General Public License
 *    along with RoboComp.  If not, see <http://www.gnu.org/licenses/>.
 */

#include "specificworker.h"

SpecificWorker::SpecificWorker(TuplePrx tprx, bool startup_check) : GenericWorker(tprx)
{
	this->startup_check_flag = startup_check;
}

/**
* \brief Default destructor
*/
SpecificWorker::~SpecificWorker()
{
	std::cout << "Destroying SpecificWorker" << std::endl;
}

bool SpecificWorker::setParams(RoboCompCommonBehavior::ParameterList params)
{
//	THE FOLLOWING IS JUST AN EXAMPLE
//	To use innerModelPath parameter you should uncomment specificmonitor.cpp readConfig method content
//	try
//	{
//		RoboCompCommonBehavior::Parameter par = params.at("InnerModelPath");
//		std::string innermodel_path = par.value;
//		innerModel = std::make_shared(innermodel_path);
//	}
//	catch(const std::exception &e) { qFatal("Error reading config params"); }
	return true;
}

void SpecificWorker::initialize(int period)
{
    std::cout << "Initialize worker" << std::endl;
    this->Period = period;
    if(this->startup_check_flag)
    {
        this->startup_check();
    }
    else
    {
        timer.start(Period);
    }
}

void SpecificWorker::compute()
{
    Eigen::Vector2f t(0.0, 0.0);
    try{
        RoboCompGenericBase::TBaseState bState;
        this->differentialrobot_proxy->getBaseState(bState);
    if(auto t=t1.get();t.has_value() ||t1.active)
    {
        auto tw=t.value();
        Eigen::Vector2f rw(bState.x,bState.z);
        Eigen::Matrix2f rot;
        rot<<std::cos(bState.alpha),-(std::sin(bState.alpha)),std::sin(bState.alpha),std::cos(bState.alpha);
        auto tr=rot*(tw-rw);
        auto beta=std::atan2( tr.x(), tr.y());
        auto dist=tr.norm();

        if (abs(beta) > 0.001) //rotar
        {
            differentialrobot_proxy->setSpeedBase(0, beta);
        } else if ( dist > 150) //avanzar
        {
            differentialrobot_proxy->setSpeedBase(1000, 0);
        } else //IDDLE
        {
            t1.set_task_finished();
            differentialrobot_proxy->setSpeedBase(0,0);
        }
    }

    }catch(const std::exception &e) { qFatal("Error reading config params"); }
}

int SpecificWorker::startup_check()
{
	std::cout << "Startup check" << std::endl;
	QTimer::singleShot(200, qApp, SLOT(quit()));
	return 0;
}


//SUBSCRIPTION to setPick method from RCISMousePicker interface
void SpecificWorker::RCISMousePicker_setPick(RoboCompRCISMousePicker::Pick myPick)
{
//subscribesToCODE
    t1.put( Eigen::Vector2f(myPick.x,myPick.z));
    std::cout<<myPick.x<<" "<<myPick.z<<std::endl;
}


/**************************************/
// From the RoboCompDifferentialRobot you can call this methods:
// this->differentialrobot_proxy->correctOdometer(...)
// this->differentialrobot_proxy->getBasePose(...)
// this->differentialrobot_proxy->getBaseState(...)
// this->differentialrobot_proxy->resetOdometer(...)
// this->differentialrobot_proxy->setOdometer(...)
// this->differentialrobot_proxy->setOdometerPose(...)
// this->differentialrobot_proxy->setSpeedBase(...)
// this->differentialrobot_proxy->stopBase(...)

/**************************************/
// From the RoboCompDifferentialRobot you can use this types:
// RoboCompDifferentialRobot::TMechParams

/**************************************/
// From the RoboCompLaser you can call this methods:
// this->laser_proxy->getLaserAndBStateData(...)
// this->laser_proxy->getLaserConfData(...)
// this->laser_proxy->getLaserData(...)

/**************************************/
// From the RoboCompLaser you can use this types:
// RoboCompLaser::LaserConfData
// RoboCompLaser::TData

/**************************************/
// From the RoboCompRCISMousePicker you can use this types:
// RoboCompRCISMousePicker::Pick

