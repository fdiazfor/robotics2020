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

enum Estado{avanzar,pared,rotar};
template <typename T>
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

    void objetivo(float dist);
    void avanzar(float threshold, RoboCompLaser::TLaserData ldata,float beta,float alpha,float dist);
    void pared(float threshold,  RoboCompLaser::TLaserData ldata);
    void rotar(float threshold,  RoboCompLaser::TLaserData ldata, float alpha, float target);

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
    std::tuple<float,float> coord;
    Target t1;
    Estado est;
};

#endif
