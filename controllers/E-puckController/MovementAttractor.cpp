void transform2Distance(cv::Mat sensorReadings)
{
  // transformiert Sensorwerte in metrische Werte
  
  // durchschnittliche min-max Werte aus der Literatur
  float min_mean[8] = {67.0834, 67.1868, 67.2067, 67.2328, 67.2380, 67.1407, 67.2426, 67.1777};
  float max_mean = 1760.0;
  
  // normalisierung und umrechnung
  for (int i=0; i<8; i++)
  {
    sensorReadings.at<float>(i) = (sensorReadings.at<float>(i) - min_mean[i]) / (max_mean - min_mean[i]);
    sensorReadings.at<float>(i) = std::min(std::max(sensorReadings.at<float>(i), 0.0f),1.0f);
    if (sensorReadings.at<float>(i) <= 0)
    {
      sensorReadings.at<float>(i) = 70;
    }
    else
    {
      sensorReadings.at<float>(i) = -9.6 * log(sensorReadings.at<float>(i)) - 1;
    }
  }
}

float f_target(float psi_target)
{
  // target forcelet
  float lambda = 0.8;
  return -lambda * sin(psi_target);
}

void f_obstacle(cv::Mat ps_distance, float forcelets[])
{
  // forcelet parameter
  float sigma = M_PI / 3;
  float beta_1 = 15.;
  float beta_2 = 12.;
  // [psi0, psi1, psi2, ...]
  float psi_obs[8] = {1.27 - M_PI / 2, 0.77 - M_PI / 2, 0. - M_PI / 2, 5.21 - M_PI / 2, 4.21 - M_PI / 2, M_PI - M_PI / 2, 2.37 - M_PI / 2, 1.87 - M_PI / 2};
  
  float lambda[8];
  for (int i = 0; i<8; i++)
  {
    lambda[i] = beta_1*exp(-ps_distance.at<float>(i)/beta_2);
    forcelets[i] = lambda[i] * psi_obs[i] * exp(-(psi_obs[i] * psi_obs[i]) / (2*sigma*sigma));
  }
}

void MovementAttractor(cv::Mat ps_distance, float psi_target, float v[])
{
  float f_obs[8];
  // parameter
  float max_v = 6.279;
  float v_0 = 0.25* max_v;

  // berechne forcelets
  f_obstacle(ps_distance, f_obs);
  f_obs[0] += 0.1*f_obs[0]; //small bias for one side
  float f_sum = 0;
  for (int i=0; i<8; i++)
  {
    f_sum += f_obs[i];  
  }
  float orientation_change = f_target(psi_target) + f_sum;
  orientation_change = std::min(std::max(orientation_change, -max_v), max_v);

  // beschr�nke �nderung auf m�gliche geschwindigkeiten
  if (abs(v_0 - orientation_change) > max_v || abs(v_0 + orientation_change) > max_v)
  {
    v[0] = ((orientation_change > 0) - (orientation_change < 0)) * max_v/2;
    v[1] = -v[0];
  }
  else
  {
    v[0] = v_0 + orientation_change;
    v[1] = v_0 - orientation_change;
  }
}
