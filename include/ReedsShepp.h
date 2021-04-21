/*********************************************************************
*
* Software License Agreement (BSD License)
*
*  Copyright (c) 2016, Guan-Horng Liu.
*  All rights reserved.
*
*  Redistribution and use in source and binary forms, with or without
*  modification, are permitted provided that the following conditions
*  are met:
*
*   * Redistributions of source code must retain the above copyright
*     notice, this list of conditions and the following disclaimer.
*   * Redistributions in binary form must reproduce the above
*     copyright notice, this list of conditions and the following
*     disclaimer in the documentation and/or other materials provided
*     with the distribution.
*   * Neither the name of the the copyright holder nor the names of its
*     contributors may be used to endorse or promote products derived
*     from this software without specific prior written permission.
*
*  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
*  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
*  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
*  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
*  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
*  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
*  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
*  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
*  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
*  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
*  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
*  POSSIBILITY OF SUCH DAMAGE.
*
* Author:  Guan-Horng Liu
*********************************************************************/

#ifndef SPACES_REEDS_SHEPP_STATE_SPACE_
#define SPACES_REEDS_SHEPP_STATE_SPACE_

#include <boost/math/constants/constants.hpp>
#include <cassert>

class ReedsSheppStateSpace
{
public:

    /** \brief The Reeds-Shepp path segment types */
    enum ReedsSheppPathSegmentType { RS_NOP=0, RS_LEFT=1, RS_STRAIGHT=2, RS_RIGHT=3 };

    /** \brief Reeds-Shepp path types */
    static const ReedsSheppPathSegmentType reedsSheppPathType[18][5];
    
    /** \brief Complete description of a ReedsShepp path */
    class ReedsSheppPath
    {
    public:
        ReedsSheppPath(const ReedsSheppPathSegmentType* type=reedsSheppPathType[0],
            double t=std::numeric_limits<double>::max(), double u=0., double v=0.,
            double w=0., double x=0.);
        
        double length() const { return totalLength_; }

        /** Path segment types */
        const ReedsSheppPathSegmentType* type_;
        /** Path segment lengths */
        double length_[5];
        /** Total length */
        double totalLength_;
    };

    ReedsSheppStateSpace(double turningRadius) : rho_(turningRadius) {}

    double distance(double q0[3], double q1[3]);

    std::vector<ReedsSheppPathSegmentType> type(double q0[3], double q1[3]);

    void sample(double q0[3], double q1[3], double step_size, double &length, std::vector<std::vector<double> > &points);

    /** \brief Return the shortest Reeds-Shepp path from SE(2) state state1 to SE(2) state state2 */
    ReedsSheppPath reedsShepp(double q0[3], double q1[3]);

protected:
    void interpolate(double q0[3], ReedsSheppPath &path, double seg, double s[3]);

    /** \brief Turning radius */
    double rho_;
};

#endif