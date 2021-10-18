#pragma once

// General
#include <limits>
#include <vector>

namespace eyeguide_local_planner
{

	/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	///  @class VelocityIterator
	///  @brief Class to get even sized samples between min and max, inluding zero
	///  if it is not included (and range goes from negative to positive).
	///  Values are clamped to [clamp_min, clamp_max] and nudged out from
	///  dead-zone [dead_min, dead_max].
	///  Make [min, max] velocities include dead-zone once overlapping,
	///  so that we're not stuck in it and can switch direction.
	///  No duplicates.
	/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	class VelocityIterator
	{
	public:
		VelocityIterator(float min, float max, int num_samples,
						 float clamp_min = -std::numeric_limits<float>::max(),
						 float clamp_max = std::numeric_limits<float>::max(),
						 float dead_min = 0, float dead_max = 0):
			current_index(0)
		{
			if (min == max) {
				samples.push_back(min);
			} else {
				num_samples = std::max(2, num_samples);

				// E.g. for 4 samples, split distance in 3 even parts
				float step_size = (max - min) / float(std::max(1, (num_samples - 1)));

				// Add dead_min once positive min is in dead-zone
				if (0 < min && min < dead_max - epsilon)
					samples.push_back(dead_min);

				float current;
				float next = min;
				for (int j = 0; j < num_samples - 1; ++j) {
					current = next;
					next += step_size;
					saveSample(current, clamp_min, clamp_max, dead_min, dead_max);
					// If 0 is among samples, this is never true.
					// Else, it inserts 0 between the positive and negative samples.
					if ((current < -epsilon) && (next > epsilon)) {
						samples.push_back(0.0);
					}
				}

				// Save exact max value
				saveSample(max, clamp_min, clamp_max, dead_min, dead_max);

				// Add dead_max once negative max is in dead-zone
				if (0 > max && max > dead_min + epsilon)
					samples.push_back(dead_max);
			}
		}

		bool saveSample(float sample, float clamp_min, float clamp_max,
						float dead_min, float dead_max) {
			// Nudge out from dead-zone [dead_min, dead_max]
			if (0 < sample && sample < dead_max - epsilon)
				sample = dead_max;
			if (0 > sample && sample > dead_min + epsilon)
				sample = dead_min;
			// Clamp to [clamp_min, clamp_max]
			sample = std::min(clamp_max, std::max(clamp_min, sample));
			// Skip duplicates
			if (samples.empty() || sample != samples.back()) {
				samples.push_back(sample);
				return true;
			}
			return false;
		}

		float getVelocity() {
			return samples.at(current_index);
		}

		VelocityIterator& operator++(int) {
			current_index++;
			return *this;
		}

		void reset() {
			current_index = 0;
		}

		bool isFinished() {
			return current_index >= samples.size();
		}

	private:
		std::vector<float> samples;
		unsigned int current_index;
		static constexpr float epsilon = 1.e-5f;
	};

} // namespace eyeguide_local_planner
