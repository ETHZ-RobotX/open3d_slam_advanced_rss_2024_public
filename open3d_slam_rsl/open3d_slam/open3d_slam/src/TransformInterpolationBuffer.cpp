/*
 * TransformInterpolationBuffer.cpp
 *
 *  Created on: Nov 9, 2021
 *      Author: jelavice
 */
#include "open3d_slam/TransformInterpolationBuffer.hpp"
#include "open3d_slam/assert.hpp"
#include "open3d_slam/output.hpp"
#include "open3d_slam/time.hpp"

#include <iostream>

namespace o3d_slam {

TransformInterpolationBuffer::TransformInterpolationBuffer() : TransformInterpolationBuffer(2000) {}

TransformInterpolationBuffer::TransformInterpolationBuffer(size_t bufferSize) {
  setSizeLimit(bufferSize);
}

void TransformInterpolationBuffer::push(const Time& time, const Transform& tf) {
  // this relies that they will be pushed in order!!!
  if (!transforms_.empty()) {
    if (time < earliest_time()) {
      std::cerr << "TransformInterpolationBuffer:: you are trying to push something earlier than the earliest measurement, this should not "
                   "happen \n";
      std::cerr << "ignnoring the mesurement \n";
      std::cerr << "Time: " << toSecondsSinceFirstMeasurement(time) << std::endl;
      std::cerr << "earliest time: " << toSecondsSinceFirstMeasurement(earliest_time()) << std::endl;
      return;
    }

    if (time < latest_time()) {
      std::cerr
          << "TransformInterpolationBuffer:: you are trying to push something out of order, this can only happen in the beginning. \n";
      std::cerr << "ignoring this mesurement \n";
      std::cerr << "Time: " << toSecondsSinceFirstMeasurement(time) << std::endl;
      std::cerr << "latest time: " << toSecondsSinceFirstMeasurement(latest_time()) << std::endl;
      return;
    }
  }
  transforms_.push_back({time, tf});
  removeOldMeasurementsIfNeeded();
  isBufferExtended = true;
}

void TransformInterpolationBuffer::applyToAllElementsInTimeInterval(const Transform& t, const Time& begin, const Time& end) {
  //	assert_ge(toUniversal(end),toUniversal(begin));
  for (auto it = transforms_.begin(); it != transforms_.end(); ++it) {
    if (it->time_ >= begin && it->time_ <= end) {
      it->transform_ = it->transform_ * t;
    }
  }
}

void TransformInterpolationBuffer::setSizeLimit(const size_t buffer_size_limit) {
  bufferSizeLimit_ = buffer_size_limit;
  removeOldMeasurementsIfNeeded();
}

void TransformInterpolationBuffer::clear() {
  transforms_.clear();
}

const TimestampedTransform& TransformInterpolationBuffer::latest_measurement(int offsetFromLastElement /*=0*/) const {
  if (empty()) {
    throw std::runtime_error("TransformInterpolationBuffer:: latest_measurement: Empty buffer");
  }

  // TODO, regardless of if a new measurement arrives we lookup through the queue, which is not nice.

  const auto lastM = std::prev(transforms_.end(), 1);
  // std::cout << "prevv: " << toSecondsSinceFirstMeasurement(prevv->time_) << std::endl;
  // std::cout << "prevv transform: " << o3d_slam::asString(prevv->transform_) << std::endl;

  if (lastM == transforms_.end()) {
    // If for some reason we are at the end of the queue, return the second to last element.
    return *(std::prev(transforms_.end(), 2));
  }

  // return transforms_.back();
  return *lastM;
}

const TimestampedTransform& TransformInterpolationBuffer::latest_offseted_measurement(int offsetFromLastElement) const {
  if (empty()) {
    throw std::runtime_error("TransformInterpolationBuffer:: latest_measurement: Empty buffer");
  }
  return *(std::prev(transforms_.end(), offsetFromLastElement + 1));
}

TimestampedTransform& TransformInterpolationBuffer::latest_measurement(int offsetFromLastElement /*=0*/) {
  if (empty()) {
    throw std::runtime_error("TransformInterpolationBuffer:: latest_measurement: Empty buffer");
  }
  return *(std::prev(transforms_.end(), offsetFromLastElement + 1));
}

bool TransformInterpolationBuffer::has(const Time& time) const {
  if (transforms_.empty()) {
    return false;
  }
  return earliest_time() <= time && time <= latest_time();
}

Transform TransformInterpolationBuffer::lookup(const Time& time) const {
  if (!has(time)) {
    throw std::runtime_error("TransformInterpolationBuffer:: Missing transform for: " + toString(time));
  }

  if (size() == 1) {
    return transforms_.front().transform_;
  }

  // just return the closest
  const auto getMeasurement =
      std::find_if(transforms_.begin(), transforms_.end(), [&time](const TimestampedTransform& tf) { return time <= tf.time_; });
  const bool isIteratorValid = getMeasurement != transforms_.end();
  if (isIteratorValid && getMeasurement->time_ == time) {
    return getMeasurement->transform_;
  }

  // An idea -> if the measurement index is the last, having "next" might be ill defined? Maybe this is whats happenning.
  if (!isTimeValid(getMeasurement->time_)) {
    const auto nexxt = std::next(getMeasurement);

    if (nexxt != transforms_.end()) {
      return nexxt->transform_;
    } else {
      const auto start2 = std::prev(getMeasurement);

      return start2->transform_;
    }
  }

  const auto start = std::prev(getMeasurement);

  //  std::cout << "buffer size: " << size() << "\n";
  //  std::cout << "left time: " << toSecondsSinceFirstMeasurement(start->time_) << "\n";
  //  std::cout << "right time: " << toSecondsSinceFirstMeasurement(getMeasurement->time_) << "\n";
  //  std::cout << "query time: " << toSecondsSinceFirstMeasurement(time) << "\n \n";
  //  std::cout << "times in buffer: \n";

  // Too dangerous, overloads the console
  // printTimesCurrentlyInBuffer();

  return interpolate(*start, *getMeasurement, time).transform_;
}

void TransformInterpolationBuffer::removeOldMeasurementsIfNeeded() {
  while (transforms_.size() > bufferSizeLimit_) {
    transforms_.pop_front();
  }
}

Time TransformInterpolationBuffer::earliest_time() const {
  if (empty()) {
    throw std::runtime_error("TransformInterpolationBuffer:: Empty buffer");
  }
  return transforms_.front().time_;
}

Time TransformInterpolationBuffer::latest_time() const {
  if (empty()) {
    throw std::runtime_error("TransformInterpolationBuffer:: Empty buffer");
  }
  return transforms_.back().time_;
}

bool TransformInterpolationBuffer::empty() const {
  return transforms_.empty();
}

size_t TransformInterpolationBuffer::size_limit() const {
  return bufferSizeLimit_;
}

size_t TransformInterpolationBuffer::size() const {
  return transforms_.size();
}

void TransformInterpolationBuffer::printTimesCurrentlyInBuffer() const {
  for (auto it = transforms_.cbegin(); it != transforms_.cend(); ++it) {
    std::cout << toSecondsSinceFirstMeasurement(it->time_) << std::endl;
  }
}

Transform getTransform(const Time& time, const TransformInterpolationBuffer& buffer) {
  if (time < buffer.earliest_time()) {
    std::cerr << "TransformInterpolationBuffer:: you are trying to get a transform that is in the past, this should not happen \n";
    return buffer.lookup(buffer.earliest_time());
  }
  if (time > buffer.latest_measurement().time_) {
    /*
    std::cerr << "TransformInterpolationBuffer:: you are trying to get a transform that is in the future, this should not happen \n";

    // Time

    std::cerr << "Time: " << toSecondsSinceFirstMeasurement(time) << std::endl;
    std::cerr << "latest_time: " << toSecondsSinceFirstMeasurement(buffer.latest_time()) << std::endl;
    std::cerr << "latest_measurement time: " << toSecondsSinceFirstMeasurement(buffer.latest_measurement().time_) << std::endl;
    std::cerr << "start_time: " << toSecondsSinceFirstMeasurement(buffer.latest_offseted_measurement(2).time_) << std::endl;

    // Time
    std::cerr << "Future Time: " << toSecondsSinceFirstMeasurement(time) << std::endl;
    std::cerr << "start_time: " << toSecondsSinceFirstMeasurement(buffer.latest_offseted_measurement(2).time_) << std::endl;
    std::cerr << "end time: " << toSecondsSinceFirstMeasurement(buffer.latest_measurement().time_) << std::endl;
    */

    auto futureTransform = extrapolate(buffer.latest_offseted_measurement(2), buffer.latest_measurement(), time);

    return futureTransform.transform_;

    // return buffer.lookup(buffer.latest_time());
  }
  return buffer.lookup(time);
}

}  // namespace o3d_slam
