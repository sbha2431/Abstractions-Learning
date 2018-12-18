#include <vision/GoalDetector.h>
#include <memory/TextLogger.h>
#include <vision/Logging.h>
#include <vision/structures/Blob.h>
#include <limits>

using namespace Eigen;

const float real_aspect_ratio = 0.5;
const float density_threshold = 0.2;
const float aspect_ratio_lower = 0.2;
const float aspect_ratio_upper = 6;
const float num_runs_min = 12;

GoalDetector::GoalDetector(DETECTOR_DECLARE_ARGS) : DETECTOR_INITIALIZE {
}

void GoalDetector::findGoal(std::map<int, std::vector<struct Blob*>> blob_map) {

  auto blobs_itr = blob_map.find(c_BLUE);
  if (blobs_itr == blob_map.end()){
    return;
  }
  auto blue_blobs = blobs_itr->second;

  // printf("Goal blobs detected %i\n", blue_blobs.size());
  float best_goal_aspect_ratio = std::numeric_limits<float>::max();
  struct Blob *best_goal = NULL;
  for(struct Blob * blue_blob : blue_blobs) {

    printf("Blue blob xi: %i yi: %i xf: %i yf: %i\n", blue_blob->xi, blue_blob->yi, blue_blob->xf, blue_blob->yf);
    float blob_density = blue_blob->correctPixelRatio * 8;
    int num_pixels = blue_blob->edgeSize;
    int height = abs(blue_blob->yi - blue_blob->yf);
    int width = abs(blue_blob->xi - blue_blob->xf);
    float aspect_ratio = float(height) / float(width);

    bool density_test = (blob_density >= density_threshold);
    bool aspect_ratio_test = (aspect_ratio >= aspect_ratio_lower 
      && aspect_ratio <= aspect_ratio_upper);

    aspect_ratio_test = true;
    bool num_runs_test = (blue_blob->lpCount >= num_runs_min);
    /*
     other heuristics - number of pixels based on distance refer to paper
    */

    // printf("density test %i (density %f) ", density_test, blob_density);
    // printf("aspect ratio test %i ", aspect_ratio_test);
    // printf("num runs test %i\n", num_runs_test);
    if (density_test && aspect_ratio_test && num_runs_test) {
      float aspect_ratio_diff = abs(aspect_ratio - real_aspect_ratio);
      if (aspect_ratio_diff <= best_goal_aspect_ratio) {
        best_goal = blue_blob;
        best_goal_aspect_ratio = aspect_ratio_diff;
      } 
    } 
  }

  if (best_goal == NULL) {
    // printf("Cant find best goal\n");
  } else {
      auto *object = &vblocks_.world_object->objects_[WO_OWN_GOAL];
      object->imageCenterX = (best_goal->xi + best_goal->xf) / 2;
      object->imageCenterY = (best_goal->yi + best_goal->yf) / 2;
      auto position = cmatrix_.getWorldPosition(object->imageCenterX, object->imageCenterY,250.0);
      object->visionBearing = cmatrix_.bearing(position);
      object->visionDistance = cmatrix_.groundDistance(position);
      object->seen = true;
      object->fromTopCamera = camera_ == Camera::TOP;
      object->upperHeight = abs(float(best_goal->yi - best_goal->yf));
      object->lowerHeight = abs(float(best_goal->xi - best_goal->xf));
      std::cout << "Goal seen" << endl;
      printf("Goal created. xi: %i xf: %i yi: %i yf: %i at a distance of %f\n", best_goal->xi, best_goal->xf, best_goal->yi, best_goal->yf, object->visionDistance);
  }
}

