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

/**
	\brief
	@author authorname
*/



#ifndef SPECIFICWORKER_H
#define SPECIFICWORKER_H

#include <genericworker.h>
#include <innermodel/innermodel.h>
#include <Eigen/Dense>
template <typename T>

enum Estado{avanzar,pared,rotar,parar};

class SpecificWorker : public GenericWorker
{
Q_OBJECT
public:
	SpecificWorker(TuplePrx tprx, bool startup_check);
	~SpecificWorker();
	bool setParams(RoboCompCommonBehavior::ParameterList params);


	void RCISMousePicker_setPick(RoboCompRCISMousePicker::Pick myPick);

public slots:
	void compute();
	int startup_check();
	void initialize(int period);

private:
	std::shared_ptr < InnerModel > innerModel;
	bool startup_check_flag;

    struct Target
    {
        T content;
        std::mutex my_mutex;
        bool active = false;

        void put(const T &data)
        {
            std::lock_guard<std::mutex> guard(my_mutex);
            content = data;   // generic type must be copy-constructable
            active = true;
        }
        std::optional<T> get()
        {
            std::lock_guard<std::mutex> guard(my_mutex);
            if(active)
                return content;
            else
                return {};
        }
        void set_task_finished()
        {
            std::lock_guard<std::mutex> guard(my_mutex);
            active = false;
        }
    };

    std::make_tuple<float,float>coord;
    Target t1;
    Estado est;
    void objetivo(float dist);
    void avanzar(float threshold, RoboCompLaser::TLaserData ldata,float beta,float alpha,float dist));
    void pared(float threshold,  RoboCompLaser::TLaserData ldata , int i, int j);
    void rotar(float threshold,  RoboCompLaser::TLaserData ldata, int i, int j, float alpha, float target);
};

#endif
