#include "PID.h"
#include <cmath>

#define MAX_HISTORY_SIZE 500

PID::PID() {}

PID::~PID() {}

void PID::Init(double Kp, double Ki, double Kd) {
    this->Kp = Kp;
    this->Ki = Ki;
    this->Kd = Kd;

    p_error = i_error = d_error = 0;
    cte_history.clear();

    prev_cte = 0;
}

void PID::UpdateError(double cte) {
    p_error = - Kp * cte;
    d_error = - Kd * (cte - prev_cte);
    prev_cte = cte;

    cte_history.push_back(cte);
    if (cte_history.size() > MAX_HISTORY_SIZE) {
        cte_history.pop_front();
    }

    i_error = -Ki * SumHistoryCte();
}

double PID::TotalError() {
    return SumHistoryCte2() / cte_history.size();
}

double PID::GetSteer() {
    double steer = p_error + d_error + i_error;

    if (steer < -1) {
        return -1;
    }

    if (steer > 1) {
        return 1;
    }

    return steer;
}

double PID::SumHistoryCte() {
    double sum_cte = 0;
    std::deque<double>::iterator it = cte_history.begin();
    while (it != cte_history.end()) {
        sum_cte += *it++;
    }

    return sum_cte;
}

double PID::SumHistoryCte2() {
    double sum_cte = 0;
    std::deque<double>::iterator it = cte_history.begin();
    while (it != cte_history.end()) {
        sum_cte += pow(*it++, 2);
    }

    return sum_cte;
}
