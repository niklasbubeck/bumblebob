#include "../include/mesh.hpp"

Mesh::Mesh(std::vector<cv::Point2d> cone_positions, std::vector<cv::Point2d> blue_cones,
           std::vector<cv::Point2d> yellow_cones, double density, double ppm, double threshold)
{
  density_ = density;
  ppm_ = ppm;
  threshold_ = threshold;

  for (int i = 0; i < yellow_cones.size(); i++)
  {
    if (abs(yellow_cones[i].x) > 5)
    {
      yellow_cones.erase(yellow_cones.begin() + i);
    }
  }

  for (int i = 0; i < blue_cones.size(); i++)
  {
    if (abs(blue_cones[i].x) > 5)
    {
      blue_cones.erase(blue_cones.begin() + i);
    }
  }

  CatmullRomSplines crs_blue(blue_cones, false);
  CatmullRomSplines crs_yellow(yellow_cones, false);

  mesh_points_ = cone_positions;
  blue_cones_ = crs_blue.getSplineLine(ppm_);
  yellow_cones_ = crs_yellow.getSplineLine(ppm_);
}

void Mesh::setDensity(int newValue)
{
  density_ = newValue;
}

std::vector<cv::Point2d> Mesh::getMeshPoints()
{
  return mesh_points_;
}

std::vector<double> Mesh::pointToDouble(std::vector<cv::Point2d> point_array)
{
  std::vector<double> cone_coords = {};
  for (int i = 0; i < point_array.size(); i++)
  {
    cone_coords.push_back(point_array[i].x);
    cone_coords.push_back(point_array[i].y);
  }
  return cone_coords;
}

std::vector<cv::Point2d> Mesh::doubleToPoint(std::vector<double> double_array)
{
  std::vector<cv::Point2d> point_array = {};
  for (int i = 0; i < double_array.size(); i += 2)
  {
    cv::Point2d point = { double_array[i], double_array[i + 1] };
    point_array.push_back(point);
  }
  return point_array;
}

int Mesh::calculateNearestPointIndex(std::vector<cv::Point2d> point_array, cv::Point2d point)
{
  int index = 0;
  double smallest_dist = 100000;
  for (int i = 0; i < point_array.size(); i++)
  {
    double distance = sqrt(pow(point_array[i].x - point.x, 2) + pow(point_array[i].y - point.y, 2));
    if (distance < smallest_dist)
    {
      smallest_dist = distance;
      index = i;
    }
  }
  return index;
}

double Mesh::calculateNearestPointDistance(std::vector<cv::Point2d> point_array, cv::Point2d point)
{
  double smallest_dist = 100000;
  for (int i = 0; i < point_array.size(); i++)
  {
    double distance = sqrt(pow(point_array[i].x - point.x, 2) + pow(point_array[i].y - point.y, 2));
    if (distance < smallest_dist)
    {
      smallest_dist = distance;
    }
  }
  return smallest_dist;
}

void Mesh::deletePointsOutside()
{
  for (int i = 0; i < mesh_points_.size(); i++)
  {
    double dist_blue = calculateNearestPointDistance(blue_cones_, mesh_points_[i]);
    double index_blue = calculateNearestPointIndex(blue_cones_, mesh_points_[i]);

    double dist_yell = calculateNearestPointDistance(yellow_cones_, mesh_points_[i]);
    double index_yell = calculateNearestPointIndex(yellow_cones_, mesh_points_[i]);

    double track_width = sqrt(pow(yellow_cones_[index_yell].x - blue_cones_[index_blue].x, 2) +
                              pow(yellow_cones_[index_yell].y - blue_cones_[index_blue].y, 2));

    if ((dist_blue + dist_yell) > track_width + threshold_)
    {
      mesh_points_.erase(mesh_points_.begin() + i);
    }
  }
  //-------------------------------

  for (int i = 0; i < mesh_points_.size(); i++)
  {
    double index_blue = calculateNearestPointIndex(blue_cones_, mesh_points_[i]);
    double index_yell = calculateNearestPointIndex(yellow_cones_, mesh_points_[i]);

    // edgecases for the first point
    if (index_blue <= 1 || index_yell <= 1)
    {
      index_yell = 2;
      index_blue = 2;
    }

    cv::Point2d blue_vec(blue_cones_[index_blue] - blue_cones_[index_blue - 1]);
    cv::Point2d yell_vec(yellow_cones_[index_yell] - yellow_cones_[index_yell - 1]);

    double angle_blue = M_PI_2 - (atan2(blue_vec.y, blue_vec.x));
    double angle_yell = M_PI_2 - (atan2(yell_vec.y, yell_vec.x));

    double relative_x_blue = mesh_points_[i].x - blue_cones_[index_blue].x;
    double relative_y_blue = mesh_points_[i].y - blue_cones_[index_blue].y;
    double relative_x_yell = mesh_points_[i].x - yellow_cones_[index_yell].x;
    double relative_y_yell = mesh_points_[i].y - yellow_cones_[index_yell].y;

    double rotated_blue = std::cos(angle_blue) * relative_x_blue - std::sin(angle_blue) * relative_y_blue;
    double rotated_yell = std::cos(angle_yell) * relative_x_yell - std::sin(angle_yell) * relative_y_yell;

    if (rotated_blue < -threshold_ && rotated_yell < -threshold_ ||
        rotated_yell > threshold_ && rotated_blue > threshold_)
    {
      mesh_points_.erase(mesh_points_.begin() + i);
    }
  }
}

void Mesh::deletePointsBetween()
{
}

void Mesh::calculateMesh()
{
  for (int i = 0; i <= density_; i++)
  {
    std::vector<double> doubled_mesh_points = pointToDouble(mesh_points_);
    delaunator::Delaunator mesh(doubled_mesh_points);

    for (int i = 0; i < mesh.triangles.size(); i += 3)
    {
      cv::Point2d first = { mesh.coords[2 * mesh.triangles[i]], mesh.coords[2 * mesh.triangles[i] + 1] };
      cv::Point2d second = { mesh.coords[2 * mesh.triangles[i + 1]], mesh.coords[2 * mesh.triangles[i + 1] + 1] };
      cv::Point2d third = { mesh.coords[2 * mesh.triangles[i + 2]], mesh.coords[2 * mesh.triangles[i + 2] + 1] };

      if (i == density_)
      {
        break;
      }

      cv::Point2d mpoints = { (first + second) / 2 };

      mesh_points_.push_back((first + second) / 2);
      mesh_points_.push_back((first + third) / 2);
      mesh_points_.push_back((second + third) / 2);

      middlepoints_.push_back((first + second) / 2);
      middlepoints_.push_back((first + third) / 2);
      middlepoints_.push_back((second + third) / 2);
    }

    mesh_points_.erase(unique(mesh_points_.begin(), mesh_points_.end()), mesh_points_.end());
    deletePointsOutside();

    std::vector<double> x = {};
    std::vector<double> y = {};
    for (int i = 0; i < mesh_points_.size(); i++)
    {
      x.push_back(mesh_points_[i].x);
      y.push_back(mesh_points_[i].y);
    }
  }
}
