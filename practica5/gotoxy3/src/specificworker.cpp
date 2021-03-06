/*
 *    Copyright (C) 2020 by jvallero & mtorocom
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
#include <Eigen/Dense>
#include <iostream>
#include <math.h>
#include "grid.h"


using namespace std;

const float landa = -0.5 / log(0.1);

/**
* \brief Default constructor
*/
SpecificWorker::SpecificWorker(TuplePrx tprx, bool startup_check) : GenericWorker(tprx) {
    this->startup_check_flag = startup_check;
}

/**
* \brief Default destructor
*/
SpecificWorker::~SpecificWorker()
{
    std::cout << "Destroying SpecificWorker" << std::endl;
}

bool SpecificWorker::setParams(RoboCompCommonBehavior::ParameterList params) {
    try {
        RoboCompCommonBehavior::Parameter par = params.at("InnerModelPath");
        std::string innermodel_path = par.value;
        innerModel = std::make_shared<InnerModel>(innermodel_path);
    }
    catch (const std::exception &e) { qFatal("Error reading config params"); }
    return true;
}

void SpecificWorker::initialize(int period)
{
    std::cout << "Initialize worker" << std::endl;

    // graphics
    graphicsView = new QGraphicsView(this);
    graphicsView->resize(this->size());
    graphicsView->setScene(&scene);
    graphicsView->setMinimumSize(400, 400);
    scene.setItemIndexMethod(QGraphicsScene::NoIndex);
    struct Dimensions {
        int TILE_SIZE = 100;
        float HMIN = -2500, VMIN = -2500, WIDTH = 5000, HEIGHT = 5000;
    };
    Dimensions dim;
    scene.setSceneRect(dim.HMIN, dim.VMIN, dim.WIDTH, dim.HEIGHT);
    graphicsView->scale(1, -1);

    graphicsView->show();

    //robot
    QPolygonF poly2;
    float size = ROBOT_LENGTH / 2.f;
    poly2 << QPoint(-size, -size)
          << QPoint(-size, size)
          << QPoint(-size / 3, size * 1.6)
          << QPoint(size / 3, size * 1.6)
          << QPoint(size, size)
          << QPoint(size, -size);
    QBrush brush;
    brush.setColor(QColor("DarkRed"));
    brush.setStyle(Qt::SolidPattern);
    robot_polygon = (QGraphicsItem *) scene.addPolygon(poly2, QPen(QColor("DarkRed")), brush);
    robot_polygon->setZValue(5);
    RoboCompGenericBase::TBaseState bState;
    try
    {
        differentialrobot_proxy->getBaseState(bState);
        robot_polygon->setRotation(qRadiansToDegrees(-bState.alpha));
        robot_polygon->setPos(bState.x, bState.z);
    }
    catch (const Ice::Exception &e) { std::cout << e.what() << std::endl; }
    graphicsView->fitInView(scene.sceneRect(), Qt::KeepAspectRatio);
    // grid
    grid.create_graphic_items(scene);
    // recorrer las cajas y poner a ocupado todos las celdas que caigan
    this->fill_grid_with_obstacles();
    // recorrer
    //
    //
    // las pared y poner las celdas a rojo
    this->fill_grid_with_walls();
    this->Period = 100;
    if (this->startup_check_flag) {
        this->startup_check();
    } else {
        timer.start(Period);
    }
}

void SpecificWorker::fill_grid_with_walls(){
    int i;
    for(i=-grid.get_Width()/2;i<grid.get_Width()/2;i++)
        grid.set_Occupied(i, grid.get_Width()/2 - 1);


    for(i=-grid.get_Width()/2;i<grid.get_Width()/2;i++)
        grid.set_Occupied(i,-grid.get_Width()/2);

    for(i=-grid.get_Width()/2 + 1;i<grid.get_Width()/2;i++)
        grid.set_Occupied(grid.get_Width()/2 - 1,i);

    for(i=-grid.get_Width()/2 + 1;i<grid.get_Width()/2;i++)
        grid.set_Occupied(-grid.get_Width()/2,i);

}

void SpecificWorker::fill_grid_with_obstacles()
{
    for(int i=1; i<10; i++)  //max number of boxes
    {
        auto caja = "caja" + QString::number(i);
        auto node = innerModel->getNode(caja);
        auto mesh = innerModel->getNode("cajaMesh" + QString::number(i));
        if(node and mesh)
        {
            auto pose = innerModel->transform("world", caja);
            auto plane = dynamic_cast<InnerModelPlane*>(mesh);
            int x = pose.x();
            int z = pose.z();
            int width = plane->depth;
            int depth = plane->width;

            /****

            AQUI EL CODIGO DE MODIFICACIÓN DEL GRID
            *****/
            for(int coordi=x - width/2;(coordi<x+ width/2);coordi++){
                for(int coordj=z- depth/2;(coordj<z+ depth/2); coordj++){
                    this->grid.set_Occupied(coordi,coordj);
                }

            }
        }
    }
}


void SpecificWorker::compute()
{
    //Coordenadas del robot
    RoboCompGenericBase::TBaseState bState;
    try { differentialrobot_proxy->getBaseState(bState); }
    catch (const Ice::Exception &e) { std::cout << e.what() << std::endl; }
    RoboCompLaser::TLaserData ldata;
    try { ldata = laser_proxy->getLaserData(); }
    catch (const Ice::Exception &e) { std::cout << e.what() << std::endl; }
    std::vector <tupla> vectorOrdenado;
    if (auto data = target_buffer.get(); data.has_value())
    {
        target = data.value();
        auto &[x, y, z] = target;
        compute_navigation_function(x, z);
        // calcular la función de navegación
        //desde el target, avanzar con un fuego
    }
    if( target_buffer.is_active())
    {
        //preuntar si ha llegado
        vectorOrdenado = dynamicWindowApproach(bState, ldata);
        //buscar el vecino más bajo en el grid
        //llamar a DWA con ese punto
    }
    draw_things(bState, ldata, vectorOrdenado);
 }

    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////


vector<SpecificWorker::tupla>
SpecificWorker::dynamicWindowApproach(RoboCompGenericBase::TBaseState bState, RoboCompLaser::TLaserData &ldata) {
    //coordenadas del target del mundo real al mundo del  robot
    Eigen::Vector2f tr = transformar_targetRW(bState);
    auto dist = tr.norm();
    if ( dist < 100 )
    {
        differentialrobot_proxy->setSpeedBase(0, 0);
        target_buffer.set_task_finished();
        std::cout << __FUNCTION__ << " At target" << std::endl;
        return {};
    }
    else
    {
        //posiciones originales del robot
        float vOrigen = bState.advVz; // Advance V
        float wOrigen = bState.rotV; // Rotation W

        //calculamos las posiciones futuras del robot y se insertan en un vector.
        std::vector <tupla> vectorPuntos = calcularPuntos(vOrigen, wOrigen);

        //quitamos los puntos futuros que nos llevan a obstaculos
        std::vector <tupla> vectorSInObs = obstaculos(vectorPuntos, bState.alpha, ldata);

        //ordenamos el vector de puntos segun la distancia
        std::vector <tupla> vectorOrdenado = ordenar(vectorSInObs, tr.x(), tr.y());

        if (vectorOrdenado.size() > 0)
        {
            auto[x, y, v, w, alpha] = obtenerMin(vectorOrdenado, tr.x(), tr.y());
            std::cout << __FUNCTION__ << " " << x << " " << y << " " << v << " " << w << " " << alpha
                      << std::endl;
            try{  differentialrobot_proxy->setSpeedBase(std::min(v / 5, 1000.f), w); }
            catch (const Ice::Exception &e) { std::cout << e.what() << std::endl; }
            draw_things(bState, ldata, vectorOrdenado);
            return vectorOrdenado;
        }
        else
        {
            std::cout << "Vector vacio" << std::endl;
            std::cout << "X:  "<<bState.x<<"     Z:  "<< bState.z <<std::endl;
            return {};
        }
    }
}

/**
* A traves de un punto calculamos las coordenadas del punto en el mundo real y las transformamos al mundo del robot
* @param bState
* @return
*/
Eigen::Vector2f SpecificWorker::transformar_targetRW(RoboCompGenericBase::TBaseState bState)
{
    // Coordenadas del target en el mundo real
    auto[x, y, z] = target;

    //Target mundo real
    Eigen::Vector2f tw(x, z);

    // Robot mundo robot
    Eigen::Vector2f rw(bState.x, bState.z);

    // Inicializamos una matriz de 2x2 en sentido horario.
    Eigen::Matrix2f rot;
    rot << cos(bState.alpha), sin(bState.alpha),
            -sin(bState.alpha), cos(bState.alpha);

    // Guardamos en un vector el resultado de la transpuesta de rot por la resta de tw - rw
    Eigen::Vector2f tr = rot.transpose() * (tw - rw);

    return tr;
}

void
    SpecificWorker::draw_things(const RoboCompGenericBase::TBaseState &bState, const RoboCompLaser::TLaserData &ldata,
                                const std::vector <tupla> &puntos) {
        //draw robot
        //innerModel->updateTransformValues("base", bState.x, 0, bState.z, 0, bState.alpha, 0);
        robot_polygon->setRotation(qRadiansToDegrees(-bState.alpha));
        robot_polygon->setPos(bState.x, bState.z);
        graphicsView->resize(this->size());

        //draw laser
        if (laser_polygon != nullptr)
            scene.removeItem(laser_polygon);
        QPolygonF poly;
        for (auto &l : ldata)
            poly << robot_polygon->mapToScene(QPointF(l.dist * sin(l.angle), l.dist * cos(l.angle)));
        QColor color("LightGreen");
        color.setAlpha(40);
        laser_polygon = scene.addPolygon(poly, QPen(color), QBrush(color));
        laser_polygon->setZValue(13);

        // draw future. Draw and arch going out from the robot
        // remove existing arcspwd
        for (auto arc: arcs_vector)
            scene.removeItem(arc);
        arcs_vector.clear();
        QColor col("Red");
        for (auto &[x, y, vx, wx, a] : puntos)
        {
            QPointF centro = robot_polygon->mapToScene(x, y);
            arcs_vector.push_back(scene.addEllipse(centro.x(), centro.y(), 20, 20, QPen(col), QBrush(col)));
        }
        if(not puntos.empty()) {
            QPointF center = robot_polygon->mapToScene(std::get<0>(puntos.front()), std::get<1>(puntos.front()));
            arcs_vector.push_back(scene.addEllipse(center.x(), center.y(), 80, 80, QPen(Qt::black), QBrush(Qt::black)));
        }
    }


/**
 * Devolvemos un vector con los puntos segun el algoritmo Dynamic Window Approach; con las posiciones originales del robot vamos calculando los futuros puntos donde deberia moverse para llegar al destino.
 * @param vOrigen
 * @param wOrigen
 * @return vector de tuplas <x,y,av,giro,angulo>
 */
    std::vector <SpecificWorker::tupla> SpecificWorker::calcularPuntos(float vOrigen, float wOrigen) {
        std::vector <tupla> vectorT;
        //Calculamos las posiciones futuras del robot y se insertan en un vector.
        for (float dt = 0.3; dt < 1; dt += 0.1) { //velocidad robot
            for (float v = 0; v <= 1000; v += 100) //advance
            {
                for (float w = -3; w <= 3; w += 0.1) //rotacion
                {
                    float vNuevo = vOrigen + v;
                    float wNuevo = wOrigen + w;

                    if (fabs(w) > 0.01)
                    {
                        // Nuevo punto posible
                        float r = vNuevo / wNuevo; //distancia desde el centro del robot al target
                        float x = r - r * cos(wNuevo * dt); //coordenada futura X
                        float y = r * sin(wNuevo * dt); //coordenada futura Z
                        float alp = wNuevo * dt; //angulo nuevo del robot

                        vectorT.emplace_back(
                                std::make_tuple(x, y, vNuevo, wNuevo, alp)); //lo añadimos al vector de tuplas
                        //  std::cout << __FUNCTION__ << " " << x << " " << y << " " << r << " " << vNuevo << " " << wNuevo << " " << std::endl;
                    }
                    else // para evitar la división por cero en el cálculo de r
                        vectorT.emplace_back(std::make_tuple(0, v * dt, vNuevo, wNuevo, wNuevo * dt));
                }
            }
        }
        return vectorT;
    }


/**
* Se devuelve un vector quitando los puntos futuros que nos llevan a obstaculos utilizando dos poligonos:
 * un poligono con los datos del laser y otro con los puntos de las coordenadas futuras del robot y sus esquinas.
 * Se comprueba si contiene las cuatro esquinas, si es verdad se inserta en el vector final, sino pasamos a otro punto.
 * @param vector
 * @param aph
 * @param ldata
 * @return
 */
    std::vector <SpecificWorker::tupla>
    SpecificWorker::obstaculos(std::vector <tupla> vector, float aph, const RoboCompLaser::TLaserData &ldata)
    {
        QPolygonF polygonF_Laser;
        const float semiancho = 210; // el semiancho del robot
        std::vector <tupla> vectorOBs;

        //poligono creado con los puntos del laser
        for (auto &l: ldata)
            polygonF_Laser << QPointF(l.dist * sin(l.angle), l.dist * cos(l.angle));

        //poligono del robot con los puntos futuros con sus esquinas
        for (auto &[x, y, a, g, ang]:vector) {
            // GENERAR UN CUADRADO CON EL CENTRO EN X, Y Y ORIENTACION ANG.
            QPolygonF polygonF_Robot;
            polygonF_Robot << QPointF(x - semiancho, y + semiancho)
                           << QPointF(x + semiancho, y + semiancho)
                           << QPointF(x + semiancho, y - semiancho)
                           << QPointF(x - semiancho, y - semiancho);
            polygonF_Robot = QTransform().rotate(aph).map(polygonF_Robot);

            //comprobacion de si los puntos del poligono del robot estan contenidos en el poligono del laser,
            //si el poligono del laser no contiene un punto del robot, no contiene alguna esquina por tanto pasamos a otro.
            bool cuatroEsquinas = true;
            for (auto &p : polygonF_Robot) {
                if (!polygonF_Laser.containsPoint(p, Qt::OddEvenFill)) {
                    cuatroEsquinas = false;
                    break;
                }
            }

            // SI contiene las 4 esquinas , metemos el valor.
            if (cuatroEsquinas) {
                vectorOBs.emplace_back(std::make_tuple(x, y, a, g, ang));
            }
        }
        return vectorOBs;
    }

/**
 * Ordenamos el vector segun distancia a las coordenadas x y z
 * @param vector
 * @param x
 * @param z
 * @return vector ordenado
 */
    std::vector <SpecificWorker::tupla> SpecificWorker::ordenar(std::vector <tupla> vector, float x, float z) {
        std::sort(vector.begin(), vector.end(), [x, z](const auto &a, const auto &b) {
            const auto &[ax, ay, ca, cw, aa] = a;
            const auto &[bx, by, ba, bw, bb] = b;
            return ((ax - x) * (ax - x) + (ay - z) * (ay - z)) < ((bx - x) * (bx - x) + (by - z) * (by - z));
        });
        return vector;
    }

///////////___________________________________///////////////
    int SpecificWorker::startup_check() {
        std::cout << "Startup check" << std::endl;
        QTimer::singleShot(200, qApp, SLOT(quit()));
        return 0;
    }

/**
 * SUBSCRIPTION to setPick method from RCISMousePicker interface
 * @param myPick
 */
    void SpecificWorker::RCISMousePicker_setPick(RoboCompRCISMousePicker::Pick myPick) {

        target_buffer.put(std::make_tuple(myPick.x, myPick.y, myPick.z)); //metemos las coordenadas con el mutex iniciado
        //Coordenadas del target
        std::cout << "x: " << myPick.x;
        std::cout << "..y: " << myPick.y;
        std::cout << "..z: " << myPick.z << std::endl;
    }

    void SpecificWorker::compute_navigation_function(float x, float y) {
        grid.resetDistances();
        int dist = 0;
        std::list<MyGrid::Value> l1 = grid.get_Neighbors(grid.get_value(x, y), dist);
        std::list<MyGrid::Value> l2 = {};
        bool end = false;
        while(!end){
            for(auto current_cell: l1){
                auto selected = grid.get_Neighbors(current_cell, dist);
                l2.insert(l2.end(), selected.begin(), selected.end());
            }
            dist++;
            end = l2.empty();
            if(l2.empty()){
                for(auto current_cell: l1){
                    std::cout<<"Vecino con coordenadas X: "<<current_cell.cx<<"    Y: "<<current_cell.cy<<std::endl;
                }
            }
            l1.swap(l2);
            l2.clear();
        }
    }

    SpecificWorker::tupla SpecificWorker::obtenerMin(std::vector<tupla> vector , float x, float z){
        std::vector<std::tuple<float, tupla>> vf;
        const float A = 0.0001, B = 1, C = 0.001;
        for(auto &t: vector){
            const auto &[ax, ay, ca, cw, aa] = t; //coordenadas x, y velocidad, giro, angulo que va a girar el robot
            float dt =  ((ax - x) * (ax - x) + (ay - z) * (ay - z));
            float dn = grid.get_Dist(ax, ay);
            float da = abs(aa);
            if(!grid.get_Occupied(ax, ay))
                vf.push_back(std::make_tuple(dt*A + dn*B + da*C, t));
        }

        auto min = std::min_element(vf.begin(), vf.end(), []( auto &a, auto &b){ return std::get<0>(a) < std::get<0>(b); });
        return std::get<1>(*min);
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

//  //posicion del robot ocupada
//        grid.set_Value(bState.x, bState.z, true);
//        //posiciones de las cajas ocupadas
//        auto caja1 = innerModel->getTransform("caja1");
//        if (caja1)
//            grid.set_Value(caja1->backtX, caja1->backtZ, true);
//
//        auto caja2 = innerModel->getTransform("caja2");
//        if (caja2)
//            grid.set_Value(caja2->backtX, caja2->backtZ, true);
//
//        auto caja3 = innerModel->getTransform("caja3");
//        if (caja3)
//            grid.set_Value(caja3->backtX, caja3->backtZ, true);
//
//        auto caja4 = innerModel->getTransform("caja4");
//        if (caja4)
//            grid.set_Value(caja4->backtX, caja4->backtZ, true);
//
//        auto caja5 = innerModel->getTransform("caja5");
//        if (caja5)
//            grid.set_Value(caja5->backtX, caja5->backtZ, true);
//
//        auto caja6 = innerModel->getTransform("caja6");
//        if (caja6)
//            grid.set_Value(caja6->backtX, caja6->backtZ, true);
