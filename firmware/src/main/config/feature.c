/*
 * This file is part of Cleanflight.
 *
 * Cleanflight is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * Cleanflight is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with Cleanflight.  If not, see <http://www.gnu.org/licenses/>.
 */

#include <stdbool.h>
#include <stdint.h>

#include "hw.h"

#include "config/feature.h"

static FAST_DATA_ZERO_INIT uint32_t activeFeaturesLatch = 0;

featureConfig_t featureConfig;

void latchActiveFeatures(void)
{
    activeFeaturesLatch = featureConfig.enabledFeatures;
}

bool featureConfigured(uint32_t mask)
{
    return (featureConfig.enabledFeatures & mask) == mask;
}

bool feature(uint32_t mask)
{
    // Check for ALL masked features
    return (activeFeaturesLatch & mask) == mask;
}

void featureSet(uint32_t mask)
{
  featureConfig.enabledFeatures |= mask;
}

void featureClear(uint32_t mask)
{
  featureConfig.enabledFeatures &= ~(mask);
}

void featureClearAll(void)
{
  featureConfig.enabledFeatures = 0;
}

uint32_t featureMask(void)
{
    return featureConfig.enabledFeatures;
}
