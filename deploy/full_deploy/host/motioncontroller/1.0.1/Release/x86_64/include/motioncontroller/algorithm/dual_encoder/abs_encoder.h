#pragma once
#include <memory>
#include <vector>
class PosSensor {
public:
  PosSensor(double max_range);
  double read(double value);

private:
  double max_range_;
};

class DoubleEncoderSystem {
public:
  DoubleEncoderSystem(double encoder1_gear_ratio, double encoder2_gear_ratio,
                      double min_end_detect_pos, double detect_accuracy,
                      double encoder1_noise = 0.0, double encoder2_noise = 0.0);
  // offset: encoder read value when end position is 0
  void SetEncoder1Offset(double offset);
  void SetEncoder2Offset(double offset);
  // dir:1->same as end;-1->reverse to end
  void SetEncoder1Dir(int dir);
  void SetEncoder2Dir(int dir);

  // need to make sure the encoder offset and dir is set correctly before call
  // this function
  double GetEndPos(double encoder1_value, double encoder2_value);

  double GetMaxDetectRound();
  double GetMinEndDetectPos();
  double GetMaxEndDetectPos();

  // return: {encoder1 allowed noise, encoder2 allowed noise}
  std::vector<double> GetAllowedEncoderNoise();

private:
  std::unique_ptr<PosSensor> enc1_ptr_;
  std::unique_ptr<PosSensor> enc2_ptr_;

  double encoder1_offset_;
  double encoder2_offset_;

  double encoder1_noise_;
  double encoder2_noise_;

  int encoder1_dir_ = 1;
  int encoder2_dir_ = 1;

  double encoder1_gear_ratio_;
  double encoder2_gear_ratio_;

  double enc1_max_range_ = 360;
  double enc2_max_range_ = 360;

  double min_end_detect_pos_ = 0.0;
  double max_end_detect_pos_ = 0.0;
  double min_end_detect_pos_from_encoder1_ = 0.0;
  double max_end_detect_pos_from_encoder1_ = 0.0;
  double min_end_detect_pos_from_encoder2_ = 0.0;
  double max_end_detect_pos_from_encoder2_ = 0.0;

  std::vector<double> vec_possible_end_pos_from_encoder1_;
  std::vector<double> vec_possible_end_pos_from_encoder2_;

  double detect_accuracy_ = 0.1;

  double max_dectect_round_ = 0.0;
};