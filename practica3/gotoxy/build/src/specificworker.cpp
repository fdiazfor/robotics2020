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

/**
* \brief Default constructor
*/
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
	//computeCODE
	//QMutexLocker locker(mutex);
	//try
	//{
	//  camera_proxy->getYImage(0,img, cState, bState);
	//  memcpy(image_gray.data, &img[0], m_width*m_height*sizeof(uchar));
	//  searchTags(image_gray);
	//}
	//catch(const Ice::Exception &e)
	//{
	//  std::cout << "Error reading from Camera" << e << std::endl;
	//}
    const float threshold = 200;
    RoboCompLaser::TLaserData ldata = laser_proxy->getLaserData();
    std::sort( ldata.begin(), ldata.end(), [](RoboCompLaser::TData a, RoboCompLaser::TData b){ return     a.dist < b.dist; });
    try{
        RoboCompDifferentialRobot::TBaseState bState;
        differentialrobot_proxy->getBaseState(bState);
    if(auto t=target.get();t.hasValue())
    {
        tw=t.value();
        Eigen::Vector2f rw(bState.x,bState.z);
        Eigen::Matrix2f rot;
        rot<<cos(bState.alpha),sin(bState.alpha),sin(bState.alpha),-cos(bState.alpha);
        auto tr=rot*(tw-rw);
        auto beta=arctg(tw.x,tw.y);
        auto dist=tr.norm();

        switch (this->est){
            case Estado::avanzar:
                this->avanzar(threshold, ldata,beta,bState.alpha);
            break;
            case Estado::pared:
                this->pared(threshold, ldata);
            break;
            case Estado::rotar:
                this->rotar(threshold, ldata, i, j, alpha, target);
            break;
            case Estado::parar:
                this->parar();
                break;
        }


    }

    }catch(const Ice::Exception &e)
}

void parar(float dist){

    if(dist <0.5){
        differentialrobot_proxy->setSpeedBase(0, 0);
    }
}

void SpecificWorker::avanzar(float threshold, RoboCompLaser::TLaserData ldata,float beta,float alpha,float dist) {
    std::cout << "________avanzar_______" << std::endl;
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
    }
    differentialrobot_proxy->setSpeedBase(1000, 0);
}

void SpecificWorker::rotar(float threshold,  RoboCompLaser::TLaserData ldata, int i, int j, float alpha, float target) {
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
    t1.put(myPick);
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

