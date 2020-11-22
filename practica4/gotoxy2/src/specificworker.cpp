
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
    this->beta = 0.0;
    this->dist = 0.0;
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
//      THE FOLLOWING IS JUST AN EXAMPLE
//      To use innerModelPath parameter you should uncomment specificmonitor.cpp readConfig method content
//      try
//      {
//              RoboCompCommonBehavior::Parameter par = params.at("InnerModelPath");
//              std::string innermodel_path = par.value;
//              innerModel = std::make_shared(innermodel_path);
//      }
//      catch(const std::exception &e) { qFatal("Error reading config params"); }
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
    try{
        RoboCompGenericBase::TBaseState bState;
        this->differentialrobot_proxy->getBaseState(bState);
        RoboCompLaser::TLaserData ldata = laser_proxy->getLaserData();
        if(auto t=this->t1.get();t.has_value() || this->t1.active)
        {
            Eigen::Vector2f obs = this->obtenerObstaculos(ldata);
            Eigen::Vector2f result (t->x() + obs.x(), t->y() + obs.y());
            this->calcular(result, bState);
            this->mover(bState, t->x(), t->y());
        }
    }catch(const std::exception &e) {qFatal("Error reading config params"); }
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

void SpecificWorker::calcular(Eigen::Vector2f result, RoboCompGenericBase::TBaseState bState) {

    Eigen::Vector2f rw(bState.x,bState.z);
    Eigen::Matrix2f rot;
    rot<<std::cos(bState.alpha),-(std::sin(bState.alpha)),std::sin(bState.alpha),std::cos(bState.alpha);
    auto tr=rot*(result-rw);
    this->beta=std::atan2(tr.x(), tr.y());
    this->dist=tr.norm();
}

Eigen::Vector2f SpecificWorker::convertirCartesianas(float dist, float angle) {
    float x, y;
    x = dist*std::sin(angle);
    y = dist*std::cos(angle);
    return Eigen::Vector2f(-x, -y);
}

Eigen::Vector2f SpecificWorker::obtenerObstaculos(RoboCompLaser::TLaserData ldata){
    float xa = 0, ya = 0;
    for (const auto &l : ldata){
        if(l.dist < 3000){
            Eigen::Vector2f obs = convertirCartesianas(l.dist, l.angle);
            xa = xa + obs.x()/(pow(l.dist, 2)/5000);
            ya = ya + obs.y()/(pow(l.dist, 2)/5000);
        }
    }
    Eigen::Vector2f resobs(xa, ya);
    return resobs;
}

void SpecificWorker::mover(RoboCompGenericBase::TBaseState bState, float xobj, float yobj){
    std::cout<<"Distancia: "<<this->dist<<" Angulo: "<<this->beta<<endl;
    float reduce_speed_turning = exp(pow(this->beta, 2)/S);
    float speed_close_target = std::fmin(this->dist/1000, 1);
    differentialrobot_proxy->setSpeedBase(MAX_ADV_SPEED*reduce_speed_turning*speed_close_target, this->beta);
    float distObj = abs(bState.x - xobj) + abs(bState.z - yobj);
    if( distObj < 300 ){
        this->t1.set_task_finished();
        this->differentialrobot_proxy->setSpeedBase(0, 0);
        cout<<"______objetivo alcanzado_____"<<endl;
    }
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

/*
	 * <!--OBSTACLES-->
		<transform id="caja1" tx="0" tz="1000" ty="0" >
			<plane id="cajaMesh1" nx="1" size="400,400,400"  texture="/home/robocomp/robocomp/files/osgModels/textures/Metal.jpg" collide="1"/>
		</transform>
		<transform id="caja2" tx="1300" tz="1200" ty="0" >
			<plane id="cajaMesh2" nx="1" size="400,400,400"  texture="/home/robocomp/robocomp/files/osgModels/textures/Metal.jpg" collide="1"/>
		</transform>-->
		<transform id="caja3" tx="-1300" tz="-1200" ty="0" >
			<plane id="cajaMesh3" nx="1" size="400,400,400"  texture="/home/robocomp/robocomp/files/osgModels/textures/Metal.jpg" collide="1"/>
		</transform>-->
		<transform id="caja4" tx="1300" tz="-1200" ty="0" >
			<plane id="cajaMesh4" nx="1" size="400,400,400"  texture="/home/robocomp/robocomp/files/osgModels/textures/Metal.jpg" collide="1"/>
		</transform>
		<transform id="caja5" tx="0" tz="-1500" ty="0" >
			<plane id="cajaMesh5" nx="1" size="400,400,400"  texture="/home/robocomp/robocomp/files/osgModels/textures/Metal.jpg" collide="1"/>
		</transform>
		<transform id="caja6" tx="-1300" tz="1200" ty="0" >
			<plane id="cajaMesh6" nx="1" size="400,400,400"  texture="/home/robocomp/robocomp/files/osgModels/textures/Metal.jpg" collide="1"/>
		</transform>
		<!-- <axes id="axis" length="1000"/>
		 -->
 */
/**************************************/
// From the RoboCompLaser you can use this types:
// RoboCompLaser::LaserConfData
// RoboCompLaser::TData

/*
 * 		<!--OBSTACLES-->
		<transform id="caja1" tx="0" tz="1000" ty="0" >
			<plane id="cajaMesh1" nx="1" size="400,400,400"  texture="/home/robocomp/robocomp/files/osgModels/textures/Metal.jpg" collide="1"/>
		</transform>
		<transform id="caja2" tx="1300" tz="1200" ty="0" >
			<plane id="cajaMesh2" nx="1" size="400,400,400"  texture="/home/robocomp/robocomp/files/osgModels/textures/Metal.jpg" collide="1"/>
		</transform>-->
		<transform id="caja3" tx="-1300" tz="-1200" ty="0" >
			<plane id="cajaMesh3" nx="1" size="400,400,400"  texture="/home/robocomp/robocomp/files/osgModels/textures/Metal.jpg" collide="1"/>
		</transform>-->
		<transform id="caja4" tx="1300" tz="-1200" ty="0" >
			<plane id="cajaMesh4" nx="1" size="400,400,400"  texture="/home/robocomp/robocomp/files/osgModels/textures/Metal.jpg" collide="1"/>
		</transform>
		<transform id="caja5" tx="0" tz="-1500" ty="0" >
			<plane id="cajaMesh5" nx="1" size="400,400,400"  texture="/home/robocomp/robocomp/files/osgModels/textures/Metal.jpg" collide="1"/>
		</transform>
		<transform id="caja6" tx="-1300" tz="1200" ty="0" >
			<plane id="cajaMesh6" nx="1" size="400,400,400"  texture="/home/robocomp/robocomp/files/osgModels/textures/Metal.jpg" collide="1"/>
		</transform>
 */
/**************************************/
// From the RoboCompRCISMousePicker you can use this types:
// RoboCompRCISMousePicker::Pick

