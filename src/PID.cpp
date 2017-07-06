#include "PID.h"

using namespace std;

PID::PID() {}

PID::~PID() {}

void PID::Init(double Kp, double Ki, double Kd, bool should_twiddle)
{
    should_twiddle_ = should_twiddle;
    params[0] = Kp;
    params[1] = Ki;
    params[2] = Kd;
    for (int i = 0; i < 3; i++)
    {
        errors[i] = 0;
        dp[i] = params[i] / 10.0;
    }
    p_i = 0;
    best_error = 1e8;
    running_error = 0;
    step = 0;
    // This is approximately once around the track in the simulator
    max_steps = 4000;
    correcting_overshoot = false;
}

double PID::CalculateSteering(double dt)
{
    double p = -params[0] * errors[0];
    double i = -params[1] * errors[1];
    double d = -params[2] * errors[2] / dt;
    double steering = p + i + d;
    return (steering < -1 ? -1 : (steering > 1 ? 1 : steering));
}

void PID::UpdateError(double cte)
{
    errors[2] = cte - errors[0]; // d
    errors[0] = cte;             // p
    errors[1] += cte;            // i

    if (should_twiddle_)
    {
        running_error += cte * cte;
        step++;
        if (step > max_steps)
        {
            running_error /= double(step);
            Twiddle();
            step = 0;
            running_error = 0;
            cout << " Kp: " << params[0] << " Ki: " << params[1] << " Kd: " << params[2] << endl;
        }
    }
}

void PID::Twiddle()
{
    if (should_twiddle_)
    {
        if (correcting_overshoot == true)
        {
            if (running_error < best_error)
            {
                best_error = running_error;
                dp[p_i] *= 1.1;
            }
            else
            {
                params[p_i] += dp[p_i];
                dp[p_i] *= 0.9;
            }
            correcting_overshoot = false;
            p_i++;
            p_i = p_i % 3;
        }
        else
        {
            if (running_error < best_error)
            {
                best_error = running_error;
                dp[p_i] *= 1.1;
                params[p_i] += dp[p_i];
                p_i++;
                p_i = p_i % 3;
            }
            else
            {
                params[p_i] -= 2 * dp[p_i];
                correcting_overshoot = true;
            }
        }
    }
}
