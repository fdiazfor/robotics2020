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
	this->est=Estado::rotar;
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
		time.start(this->Period);
	}
}

void SpecificWorker::compute()
{
    const float threshold = 200;
    std::make_tuple<float,float> t;
    RoboCompLaser::TLaserData ldata = laser_proxy->getLaserData();
    std::sort( ldata.begin(), ldata.end(), [](RoboCompLaser::TData a, RoboCompLaser::TData b){ return     a.dist < b.dist; });
    try{
        RoboCompGenericBase::TBaseState& bState;
        this->differentialrobot_proxy->getBaseState(bState);
    if(auto t=t1.get();t.hasValue()||t1.active)
    {
        auto tw=t.value();
        std::cout<<tw.x" "tw.z<<std::endl;
        Eigen::Vector2f rw(bState.x,bState.z);
        Eigen::Matrix2f rot;
        rot<<std::cos(bState.alpha),std::sin(bState.alpha),tsd::sin(bState.alpha),-(std::cos(bState.alpha));
        auto tr=rot*(tw-rw);
        auto beta=std::atan(tw.x,tw.y);
        auto dist=tr.norm();

        switch (this->est){
            case Estado::avanzar:
                this->avanzar(threshold, ldata,beta,bState.alpha);
            break;
            case Estado::pared:
                this->pared(threshold, ldata);
            break;
            case Estado::rotar:
                this->rotar(threshold, ldata, bState.alpha, target);
            break;
            case Estado::objetivo:
                this->objetivo();
                break;

        }


    }

    }catch(const Ice::Exception &e)
}

void SpecificWorker::avanzar(float threshold, RoboCompLaser::TLaserData ldata,float beta,float alpha,float dist) {
    if (threshold > ldata.front().dist) {
        this->est = Estado::pared;
        differentialrobot_proxy->setSpeedBase(0, 0);
        return;
    } else if (abs(beta-alpha)>0.01) {
        this->est = Estado::rotar;
        differentialrobot_proxy->setSpeedBase(0, 0);
        return;
    }
    else if(dist <0.5){
        differentialrobot_proxy->setSpeedBase(0, 0);
        this->est = Estado::objetivo;
    }
    differentialrobot_proxy->setSpeedBase(1000, 0);
}

void SpecificWorker::rotar(float threshold,  RoboCompLaser::TLaserData ldata, float alpha, float target) {
    if (threshold > ldata.front().dist){
        differentialrobot_proxy->setSpeedBase(0, 0);
        this->est = Estado::pared;
        return;
    }else if(abs((alpha- target)) < 0.01 ){
        differentialrobot_proxy->setSpeedBase(0, 0);
        this->est = Estado::avanzar;
        return;
    }
    differentialrobot_proxy->setSpeedBase(5, 2);
}

void SpecificWorker::pared(float threshold,  RoboCompLaser::TLaserData ldata ) {
    float rot = 2;
    if (threshold < ldata.front().dist){
        differentialrobot_proxy->setSpeedBase(0, 0);
        this->est = Estado::avanzar;
        return;
    }
    differentialrobot_proxy->setSpeedBase(5, rot);
}

void SpecificWorker::objetivo(){
    t1.set_task_finished();
    differentialrobot_proxy->setSpeedBase(0,0);
    this->est = Estado::avanzar;
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
    t1.put(std::make_tuple(myPick.x,myPick.z));
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

