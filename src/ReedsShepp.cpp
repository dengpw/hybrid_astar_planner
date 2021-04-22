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


#include "ReedsShepp.h"
#include <boost/math/constants/constants.hpp>


namespace
{
    // The comments, variable names, etc. use the nomenclature from the Reeds & Shepp paper.

    const double pi = boost::math::constants::pi<double>();
    const double twopi = 2. * pi;
    const double RS_EPS = 1e-6;
    const double ZERO = 10*std::numeric_limits<double>::epsilon();

    inline double mod2pi(double x)
    {
        double v = fmod(x, twopi);
        if (v < -pi)
            v += twopi;
        else
            if (v > pi)
                v -= twopi;
        return v;
    }
    inline void polar(double x, double y, double &r, double &theta)
    {
        r = sqrt(x*x + y*y);
        theta = atan2(y, x);
    }
    inline void tauOmega(double u, double v, double xi, double eta, double phi, double &tau, double &omega)
    {
        double delta = mod2pi(u-v), A = sin(u) - sin(delta), B = cos(u) - cos(delta) - 1.;
        double t1 = atan2(eta*A - xi*B, xi*A + eta*B), t2 = 2. * (cos(delta) - cos(v) - cos(u)) + 3;
        tau = (t2<0) ? mod2pi(t1+pi) : mod2pi(t1);
        omega = mod2pi(tau - u + v - phi) ;
    }

    // formula 8.1 in Reeds-Shepp paper
    inline bool LpSpLp(double x, double y, double phi, double &t, double &u, double &v)
    {
        polar(x - sin(phi), y - 1. + cos(phi), u, t);
        if (t >= -ZERO)
        {
            v = mod2pi(phi - t);
            if (v >= -ZERO)
            {
                assert(fabs(u*cos(t) + sin(phi) - x) < RS_EPS);
                assert(fabs(u*sin(t) - cos(phi) + 1 - y) < RS_EPS);
                assert(fabs(mod2pi(t+v - phi)) < RS_EPS);
                return true;
            }
        }
        return false;
    }
    // formula 8.2
    inline bool LpSpRp(double x, double y, double phi, double &t, double &u, double &v)
    {
        double t1, u1;
        polar(x + sin(phi), y - 1. - cos(phi), u1, t1);
        u1 = u1*u1;
        if (u1 >= 4.)
        {
            double theta;
            u = sqrt(u1 - 4.);
            theta = atan2(2., u);
            t = mod2pi(t1 + theta);
            v = mod2pi(t - phi);
            assert(fabs(2*sin(t) + u*cos(t) - sin(phi) - x) < RS_EPS);
            assert(fabs(-2*cos(t) + u*sin(t) + cos(phi) + 1 - y) < RS_EPS);
            assert(fabs(mod2pi(t-v - phi)) < RS_EPS);
            return t>=-ZERO && v>=-ZERO;
        }
        return false;
    }
    void CSC(double x, double y, double phi, ReedsSheppStateSpace::ReedsSheppPath &path)
    {
        double t, u, v, Lmin = path.length(), L;
        if (LpSpLp(x, y, phi, t, u, v) && Lmin > (L = fabs(t) + fabs(u) + fabs(v)))
        {
            path = ReedsSheppStateSpace::ReedsSheppPath(
                ReedsSheppStateSpace::reedsSheppPathType[14], t, u, v);
            Lmin = L;
        }
        if (LpSpLp(-x, y, -phi, t, u, v) && Lmin > (L = fabs(t) + fabs(u) + fabs(v))) // timeflip
        {
            path = ReedsSheppStateSpace::ReedsSheppPath(
                ReedsSheppStateSpace::reedsSheppPathType[14], -t, -u, -v);
            Lmin = L;
        }
        if (LpSpLp(x, -y, -phi, t, u, v) && Lmin > (L = fabs(t) + fabs(u) + fabs(v))) // reflect
        {
            path = ReedsSheppStateSpace::ReedsSheppPath(
                ReedsSheppStateSpace::reedsSheppPathType[15], t, u, v);
            Lmin = L;
        }
        if (LpSpLp(-x, -y, phi, t, u, v) && Lmin > (L = fabs(t) + fabs(u) + fabs(v))) // timeflip + reflect
        {
            path = ReedsSheppStateSpace::ReedsSheppPath(
                ReedsSheppStateSpace::reedsSheppPathType[15], -t, -u, -v);
            Lmin = L;
        }

        if (LpSpRp(x, y, phi, t, u, v) && Lmin > (L = fabs(t) + fabs(u) + fabs(v)))
        {
            path = ReedsSheppStateSpace::ReedsSheppPath(
                ReedsSheppStateSpace::reedsSheppPathType[12], t, u, v);
            Lmin = L;
        }
        if (LpSpRp(-x, y, -phi, t, u, v) && Lmin > (L = fabs(t) + fabs(u) + fabs(v))) // timeflip
        {
            path = ReedsSheppStateSpace::ReedsSheppPath(
                ReedsSheppStateSpace::reedsSheppPathType[12], -t, -u, -v);
            Lmin = L;
        }
        if (LpSpRp(x, -y, -phi, t, u, v) && Lmin > (L = fabs(t) + fabs(u) + fabs(v))) // reflect
        {
            path = ReedsSheppStateSpace::ReedsSheppPath(
                ReedsSheppStateSpace::reedsSheppPathType[13], t, u, v);
            Lmin = L;
        }
        if (LpSpRp(-x, -y, phi, t, u, v) && Lmin > (L = fabs(t) + fabs(u) + fabs(v))) // timeflip + reflect
            path = ReedsSheppStateSpace::ReedsSheppPath(
                ReedsSheppStateSpace::reedsSheppPathType[13], -t, -u, -v);
    }
    // formula 8.3 / 8.4  *** TYPO IN PAPER ***
    inline bool LpRmL(double x, double y, double phi, double &t, double &u, double &v)
    {
        double xi = x - sin(phi), eta = y - 1. + cos(phi), u1, theta;
        polar(xi, eta, u1, theta);
        if (u1 <= 4.)
        {
            u = -2.*asin(.25 * u1);
            t = mod2pi(theta + .5 * u + pi);
            v = mod2pi(phi - t + u);
            assert(fabs(2*(sin(t) - sin(t-u)) + sin(phi) - x) < RS_EPS);
            assert(fabs(2*(-cos(t) + cos(t-u)) - cos(phi) + 1 - y) < RS_EPS);
            assert(fabs(mod2pi(t-u+v - phi)) < RS_EPS);
            return t>=-ZERO && u<=ZERO;
        }
        return false;
    }
    void CCC(double x, double y, double phi, ReedsSheppStateSpace::ReedsSheppPath &path)
    {
        double t, u, v, Lmin = path.length(), L;
        if (LpRmL(x, y, phi, t, u, v) && Lmin > (L = fabs(t) + fabs(u) + fabs(v)))
        {
            path = ReedsSheppStateSpace::ReedsSheppPath(
                ReedsSheppStateSpace::reedsSheppPathType[0], t, u, v);
            Lmin = L;
        }
        if (LpRmL(-x, y, -phi, t, u, v) && Lmin > (L = fabs(t) + fabs(u) + fabs(v))) // timeflip
        {
            path = ReedsSheppStateSpace::ReedsSheppPath(
                ReedsSheppStateSpace::reedsSheppPathType[0], -t, -u, -v);
            Lmin = L;
        }
        if (LpRmL(x, -y, -phi, t, u, v) && Lmin > (L = fabs(t) + fabs(u) + fabs(v))) // reflect
        {
            path = ReedsSheppStateSpace::ReedsSheppPath(
                ReedsSheppStateSpace::reedsSheppPathType[1], t, u, v);
            Lmin = L;
        }
        if (LpRmL(-x, -y, phi, t, u, v) && Lmin > (L = fabs(t) + fabs(u) + fabs(v))) // timeflip + reflect
        {
            path = ReedsSheppStateSpace::ReedsSheppPath(
                ReedsSheppStateSpace::reedsSheppPathType[1], -t, -u, -v);
            Lmin = L;
        }

        // backwards
        double xb = x*cos(phi) + y*sin(phi), yb = x*sin(phi) - y*cos(phi);
        if (LpRmL(xb, yb, phi, t, u, v) && Lmin > (L = fabs(t) + fabs(u) + fabs(v)))
        {
            path = ReedsSheppStateSpace::ReedsSheppPath(
                ReedsSheppStateSpace::reedsSheppPathType[0], v, u, t);
            Lmin = L;
        }
        if (LpRmL(-xb, yb, -phi, t, u, v) && Lmin > (L = fabs(t) + fabs(u) + fabs(v))) // timeflip
        {
            path = ReedsSheppStateSpace::ReedsSheppPath(
                ReedsSheppStateSpace::reedsSheppPathType[0], -v, -u, -t);
            Lmin = L;
        }
        if (LpRmL(xb, -yb, -phi, t, u, v) && Lmin > (L = fabs(t) + fabs(u) + fabs(v))) // reflect
        {
            path = ReedsSheppStateSpace::ReedsSheppPath(
                ReedsSheppStateSpace::reedsSheppPathType[1], v, u, t);
            Lmin = L;
        }
        if (LpRmL(-xb, -yb, phi, t, u, v) && Lmin > (L = fabs(t) + fabs(u) + fabs(v))) // timeflip + reflect
            path = ReedsSheppStateSpace::ReedsSheppPath(
                ReedsSheppStateSpace::reedsSheppPathType[1], -v, -u, -t);
    }
    // formula 8.7
    inline bool LpRupLumRm(double x, double y, double phi, double &t, double &u, double &v)
    {
        double xi = x + sin(phi), eta = y - 1. - cos(phi), rho = .25 * (2. + sqrt(xi*xi + eta*eta));
        if (rho <= 1.)
        {
            u = acos(rho);
            tauOmega(u, -u, xi, eta, phi, t, v);
            assert(fabs(2*(sin(t)-sin(t-u)+sin(t-2*u))-sin(phi) - x) < RS_EPS);
            assert(fabs(2*(-cos(t)+cos(t-u)-cos(t-2*u))+cos(phi)+1 - y) < RS_EPS);
            assert(fabs(mod2pi(t-2*u-v - phi)) < RS_EPS);
            return t>=-ZERO && v<=ZERO;
        }
        return false;
    }
    // formula 8.8
    inline bool LpRumLumRp(double x, double y, double phi, double &t, double &u, double &v)
    {
        double xi = x + sin(phi), eta = y - 1. - cos(phi), rho = (20. - xi*xi - eta*eta) / 16.;
        if (rho>=0 && rho<=1)
        {
            u = -acos(rho);
            if (u >= -.5 * pi)
            {
                tauOmega(u, u, xi, eta, phi, t, v);
                assert(fabs(4*sin(t)-2*sin(t-u)-sin(phi) - x) < RS_EPS);
                assert(fabs(-4*cos(t)+2*cos(t-u)+cos(phi)+1 - y) < RS_EPS);
                assert(fabs(mod2pi(t-v - phi)) < RS_EPS);
                return t>=-ZERO && v>=-ZERO;
            }
        }
        return false;
    }
    void CCCC(double x, double y, double phi, ReedsSheppStateSpace::ReedsSheppPath &path)
    {
        double t, u, v, Lmin = path.length(), L;
        if (LpRupLumRm(x, y, phi, t, u, v) && Lmin > (L = fabs(t) + 2.*fabs(u) + fabs(v)))
        {
            path = ReedsSheppStateSpace::ReedsSheppPath(
                ReedsSheppStateSpace::reedsSheppPathType[2], t, u, -u, v);
            Lmin = L;
        }
        if (LpRupLumRm(-x, y, -phi, t, u, v) && Lmin > (L = fabs(t) + 2.*fabs(u) + fabs(v))) // timeflip
        {
            path = ReedsSheppStateSpace::ReedsSheppPath(
                ReedsSheppStateSpace::reedsSheppPathType[2], -t, -u, u, -v);
            Lmin = L;
        }
        if (LpRupLumRm(x, -y, -phi, t, u, v) && Lmin > (L = fabs(t) + 2.*fabs(u) + fabs(v))) // reflect
        {
            path = ReedsSheppStateSpace::ReedsSheppPath(
                ReedsSheppStateSpace::reedsSheppPathType[3], t, u, -u, v);
            Lmin = L;
        }
        if (LpRupLumRm(-x, -y, phi, t, u, v) && Lmin > (L = fabs(t) + 2.*fabs(u) + fabs(v))) // timeflip + reflect
        {
            path = ReedsSheppStateSpace::ReedsSheppPath(
                ReedsSheppStateSpace::reedsSheppPathType[3], -t, -u, u, -v);
            Lmin = L;
        }

        if (LpRumLumRp(x, y, phi, t, u, v) && Lmin > (L = fabs(t) + 2.*fabs(u) + fabs(v)))
        {
            path = ReedsSheppStateSpace::ReedsSheppPath(
                ReedsSheppStateSpace::reedsSheppPathType[2], t, u, u, v);
            Lmin = L;
        }
        if (LpRumLumRp(-x, y, -phi, t, u, v) && Lmin > (L = fabs(t) + 2.*fabs(u) + fabs(v))) // timeflip
        {
            path = ReedsSheppStateSpace::ReedsSheppPath(
                ReedsSheppStateSpace::reedsSheppPathType[2], -t, -u, -u, -v);
            Lmin = L;
        }
        if (LpRumLumRp(x, -y, -phi, t, u, v) && Lmin > (L = fabs(t) + 2.*fabs(u) + fabs(v))) // reflect
        {
            path = ReedsSheppStateSpace::ReedsSheppPath(
                ReedsSheppStateSpace::reedsSheppPathType[3], t, u, u, v);
            Lmin = L;
        }
        if (LpRumLumRp(-x, -y, phi, t, u, v) && Lmin > (L = fabs(t) + 2.*fabs(u) + fabs(v))) // timeflip + reflect
            path = ReedsSheppStateSpace::ReedsSheppPath(
                ReedsSheppStateSpace::reedsSheppPathType[3], -t, -u, -u, -v);
    }
    // formula 8.9
    inline bool LpRmSmLm(double x, double y, double phi, double &t, double &u, double &v)
    {
        double xi = x - sin(phi), eta = y - 1. + cos(phi), rho, theta;
        polar(xi, eta, rho, theta);
        if (rho >= 2.)
        {
            double r = sqrt(rho*rho - 4.);
            u = 2. - r;
            t = mod2pi(theta + atan2(r, -2.));
            v = mod2pi(phi - .5*pi - t);
            assert(fabs(2*(sin(t)-cos(t))-u*sin(t)+sin(phi) - x) < RS_EPS);
            assert(fabs(-2*(sin(t)+cos(t))+u*cos(t)-cos(phi)+1 - y) < RS_EPS);
            assert(fabs(mod2pi(t+pi/2+v-phi)) < RS_EPS);
            return t>=-ZERO && u<=ZERO && v<=ZERO;
        }
        return false;
    }
    // formula 8.10
    inline bool LpRmSmRm(double x, double y, double phi, double &t, double &u, double &v)
    {
        double xi = x + sin(phi), eta = y - 1. - cos(phi), rho, theta;
        polar(-eta, xi, rho, theta);
        if (rho >= 2.)
        {
            t = theta;
            u = 2. - rho;
            v = mod2pi(t + .5*pi - phi);
            assert(fabs(2*sin(t)-cos(t-v)-u*sin(t) - x) < RS_EPS);
            assert(fabs(-2*cos(t)-sin(t-v)+u*cos(t)+1 - y) < RS_EPS);
            assert(fabs(mod2pi(t+pi/2-v-phi)) < RS_EPS);
            return t>=-ZERO && u<=ZERO && v<=ZERO;
        }
        return false;
    }
    void CCSC(double x, double y, double phi, ReedsSheppStateSpace::ReedsSheppPath &path)
    {
        double t, u, v, Lmin = path.length() - .5*pi, L;
        if (LpRmSmLm(x, y, phi, t, u, v) && Lmin > (L = fabs(t) + fabs(u) + fabs(v)))
        {
            path = ReedsSheppStateSpace::ReedsSheppPath(
                ReedsSheppStateSpace::reedsSheppPathType[4], t, -.5*pi, u, v);
            Lmin = L;
        }
        if (LpRmSmLm(-x, y, -phi, t, u, v) && Lmin > (L = fabs(t) + fabs(u) + fabs(v))) // timeflip
        {
            path = ReedsSheppStateSpace::ReedsSheppPath(
                ReedsSheppStateSpace::reedsSheppPathType[4], -t, .5*pi, -u, -v);
            Lmin = L;
        }
        if (LpRmSmLm(x, -y, -phi, t, u, v) && Lmin > (L = fabs(t) + fabs(u) + fabs(v))) // reflect
        {
            path = ReedsSheppStateSpace::ReedsSheppPath(
                ReedsSheppStateSpace::reedsSheppPathType[5], t, -.5*pi, u, v);
            Lmin = L;
        }
        if (LpRmSmLm(-x, -y, phi, t, u, v) && Lmin > (L = fabs(t) + fabs(u) + fabs(v))) // timeflip + reflect
        {
            path = ReedsSheppStateSpace::ReedsSheppPath(
                ReedsSheppStateSpace::reedsSheppPathType[5], -t, .5*pi, -u, -v);
            Lmin = L;
        }

        if (LpRmSmRm(x, y, phi, t, u, v) && Lmin > (L = fabs(t) + fabs(u) + fabs(v)))
        {
            path = ReedsSheppStateSpace::ReedsSheppPath(
                ReedsSheppStateSpace::reedsSheppPathType[8], t, -.5*pi, u, v);
            Lmin = L;
        }
        if (LpRmSmRm(-x, y, -phi, t, u, v) && Lmin > (L = fabs(t) + fabs(u) + fabs(v))) // timeflip
        {
            path = ReedsSheppStateSpace::ReedsSheppPath(
                ReedsSheppStateSpace::reedsSheppPathType[8], -t, .5*pi, -u, -v);
            Lmin = L;
        }
        if (LpRmSmRm(x, -y, -phi, t, u, v) && Lmin > (L = fabs(t) + fabs(u) + fabs(v))) // reflect
        {
            path = ReedsSheppStateSpace::ReedsSheppPath(
                ReedsSheppStateSpace::reedsSheppPathType[9], t, -.5*pi, u, v);
            Lmin = L;
        }
        if (LpRmSmRm(-x, -y, phi, t, u, v) && Lmin > (L = fabs(t) + fabs(u) + fabs(v))) // timeflip + reflect
        {
            path = ReedsSheppStateSpace::ReedsSheppPath(
                ReedsSheppStateSpace::reedsSheppPathType[9], -t, .5*pi, -u, -v);
            Lmin = L;
        }

        // backwards
        double xb = x*cos(phi) + y*sin(phi), yb = x*sin(phi) - y*cos(phi);
        if (LpRmSmLm(xb, yb, phi, t, u, v) && Lmin > (L = fabs(t) + fabs(u) + fabs(v)))
        {
            path = ReedsSheppStateSpace::ReedsSheppPath(
                ReedsSheppStateSpace::reedsSheppPathType[6], v, u, -.5*pi, t);
            Lmin = L;
        }
        if (LpRmSmLm(-xb, yb, -phi, t, u, v) && Lmin > (L = fabs(t) + fabs(u) + fabs(v))) // timeflip
        {
            path = ReedsSheppStateSpace::ReedsSheppPath(
                ReedsSheppStateSpace::reedsSheppPathType[6], -v, -u, .5*pi, -t);
            Lmin = L;
        }
        if (LpRmSmLm(xb, -yb, -phi, t, u, v) && Lmin > (L = fabs(t) + fabs(u) + fabs(v))) // reflect
        {
            path = ReedsSheppStateSpace::ReedsSheppPath(
                ReedsSheppStateSpace::reedsSheppPathType[7], v, u, -.5*pi, t);
            Lmin = L;
        }
        if (LpRmSmLm(-xb, -yb, phi, t, u, v) && Lmin > (L = fabs(t) + fabs(u) + fabs(v))) // timeflip + reflect
        {
            path = ReedsSheppStateSpace::ReedsSheppPath(
                ReedsSheppStateSpace::reedsSheppPathType[7], -v, -u, .5*pi, -t);
            Lmin = L;
        }

        if (LpRmSmRm(xb, yb, phi, t, u, v) && Lmin > (L = fabs(t) + fabs(u) + fabs(v)))
        {
            path = ReedsSheppStateSpace::ReedsSheppPath(
                ReedsSheppStateSpace::reedsSheppPathType[10], v, u, -.5*pi, t);
            Lmin = L;
        }
        if (LpRmSmRm(-xb, yb, -phi, t, u, v) && Lmin > (L = fabs(t) + fabs(u) + fabs(v))) // timeflip
        {
            path = ReedsSheppStateSpace::ReedsSheppPath(
                ReedsSheppStateSpace::reedsSheppPathType[10], -v, -u, .5*pi, -t);
            Lmin = L;
        }
        if (LpRmSmRm(xb, -yb, -phi, t, u, v) && Lmin > (L = fabs(t) + fabs(u) + fabs(v))) // reflect
        {
            path = ReedsSheppStateSpace::ReedsSheppPath(
                ReedsSheppStateSpace::reedsSheppPathType[11], v, u, -.5*pi, t);
            Lmin = L;
        }
        if (LpRmSmRm(-xb, -yb, phi, t, u, v) && Lmin > (L = fabs(t) + fabs(u) + fabs(v))) // timeflip + reflect
            path = ReedsSheppStateSpace::ReedsSheppPath(
                ReedsSheppStateSpace::reedsSheppPathType[11], -v, -u, .5*pi, -t);
    }
    // formula 8.11 *** TYPO IN PAPER ***
    inline bool LpRmSLmRp(double x, double y, double phi, double &t, double &u, double &v)
    {
        double xi = x + sin(phi), eta = y - 1. - cos(phi), rho, theta;
        polar(xi, eta, rho, theta);
        if (rho >= 2.)
        {
            u = 4. - sqrt(rho*rho - 4.);
            if (u <= ZERO)
            {
                t = mod2pi(atan2((4-u)*xi -2*eta, -2*xi + (u-4)*eta));
                v = mod2pi(t - phi);
                assert(fabs(4*sin(t)-2*cos(t)-u*sin(t)-sin(phi) - x) < RS_EPS);
                assert(fabs(-4*cos(t)-2*sin(t)+u*cos(t)+cos(phi)+1 - y) < RS_EPS);
                assert(fabs(mod2pi(t-v-phi)) < RS_EPS);
                return t>=-ZERO && v>=-ZERO;
            }
        }
        return false;
    }
    void CCSCC(double x, double y, double phi, ReedsSheppStateSpace::ReedsSheppPath &path)
    {
        double t, u, v, Lmin = path.length() - pi, L;
        if (LpRmSLmRp(x, y, phi, t, u, v) && Lmin > (L = fabs(t) + fabs(u) + fabs(v)))
        {
            path = ReedsSheppStateSpace::ReedsSheppPath(
                ReedsSheppStateSpace::reedsSheppPathType[16], t, -.5*pi, u, -.5*pi, v);
            Lmin = L;
        }
        if (LpRmSLmRp(-x, y, -phi, t, u, v) && Lmin > (L = fabs(t) + fabs(u) + fabs(v))) // timeflip
        {
            path = ReedsSheppStateSpace::ReedsSheppPath(
                ReedsSheppStateSpace::reedsSheppPathType[16], -t, .5*pi, -u, .5*pi, -v);
            Lmin = L;
        }
        if (LpRmSLmRp(x, -y, -phi, t, u, v) && Lmin > (L = fabs(t) + fabs(u) + fabs(v))) // reflect
        {
            path = ReedsSheppStateSpace::ReedsSheppPath(
                ReedsSheppStateSpace::reedsSheppPathType[17], t, -.5*pi, u, -.5*pi, v);
            Lmin = L;
        }
        if (LpRmSLmRp(-x, -y, phi, t, u, v) && Lmin > (L = fabs(t) + fabs(u) + fabs(v))) // timeflip + reflect
            path = ReedsSheppStateSpace::ReedsSheppPath(
                ReedsSheppStateSpace::reedsSheppPathType[17], -t, .5*pi, -u, .5*pi, -v);
    }

    ReedsSheppStateSpace::ReedsSheppPath reedsShepp(double x, double y, double phi)
    {
        ReedsSheppStateSpace::ReedsSheppPath path;
        CSC(x, y, phi, path);
        CCC(x, y, phi, path);
        CCCC(x, y, phi, path);
        CCSC(x, y, phi, path);
        CCSCC(x, y, phi, path);
        return path;
    }
}

const ReedsSheppStateSpace::ReedsSheppPathSegmentType
ReedsSheppStateSpace::reedsSheppPathType[18][5] = {
    { RS_LEFT, RS_RIGHT, RS_LEFT, RS_NOP, RS_NOP },             // 0
    { RS_RIGHT, RS_LEFT, RS_RIGHT, RS_NOP, RS_NOP },            // 1
    { RS_LEFT, RS_RIGHT, RS_LEFT, RS_RIGHT, RS_NOP },           // 2
    { RS_RIGHT, RS_LEFT, RS_RIGHT, RS_LEFT, RS_NOP },           // 3
    { RS_LEFT, RS_RIGHT, RS_STRAIGHT, RS_LEFT, RS_NOP },        // 4
    { RS_RIGHT, RS_LEFT, RS_STRAIGHT, RS_RIGHT, RS_NOP },       // 5
    { RS_LEFT, RS_STRAIGHT, RS_RIGHT, RS_LEFT, RS_NOP },        // 6
    { RS_RIGHT, RS_STRAIGHT, RS_LEFT, RS_RIGHT, RS_NOP },       // 7
    { RS_LEFT, RS_RIGHT, RS_STRAIGHT, RS_RIGHT, RS_NOP },       // 8
    { RS_RIGHT, RS_LEFT, RS_STRAIGHT, RS_LEFT, RS_NOP },        // 9
    { RS_RIGHT, RS_STRAIGHT, RS_RIGHT, RS_LEFT, RS_NOP },       // 10
    { RS_LEFT, RS_STRAIGHT, RS_LEFT, RS_RIGHT, RS_NOP },        // 11
    { RS_LEFT, RS_STRAIGHT, RS_RIGHT, RS_NOP, RS_NOP },         // 12
    { RS_RIGHT, RS_STRAIGHT, RS_LEFT, RS_NOP, RS_NOP },         // 13
    { RS_LEFT, RS_STRAIGHT, RS_LEFT, RS_NOP, RS_NOP },          // 14
    { RS_RIGHT, RS_STRAIGHT, RS_RIGHT, RS_NOP, RS_NOP },        // 15
    { RS_LEFT, RS_RIGHT, RS_STRAIGHT, RS_LEFT, RS_RIGHT },      // 16
    { RS_RIGHT, RS_LEFT, RS_STRAIGHT, RS_RIGHT, RS_LEFT }       // 17
};

ReedsSheppStateSpace::ReedsSheppPath::ReedsSheppPath(const ReedsSheppPathSegmentType* type,
    double t, double u, double v, double w, double x)
    : type_(type)
{
    length_[0] = t; length_[1] = u; length_[2] = v; length_[3] = w; length_[4] = x;
    totalLength_ = fabs(t) + fabs(u) + fabs(v) + fabs(w) + fabs(x);
}


double ReedsSheppStateSpace::distance(double q0[3], double q1[3])
{
    return rho_ * reedsShepp(q0, q1).length();
}

ReedsSheppStateSpace::ReedsSheppPath ReedsSheppStateSpace::reedsShepp(double q0[3], double q1[3])
{
    double dx = q1[0] - q0[0], dy = q1[1] - q0[1], dth = q1[2] - q0[2];
    double c = cos(q0[2]), s = sin(q0[2]);
    double x = c*dx + s*dy, y = -s*dx + c*dy;
    return ::reedsShepp(x/rho_, y/rho_, dth);
}

std::vector<ReedsSheppStateSpace::ReedsSheppPathSegmentType> ReedsSheppStateSpace::type(double q0[3], double q1[3])
{
    ReedsSheppPath path = reedsShepp(q0, q1);
    std::vector<ReedsSheppStateSpace::ReedsSheppPathSegmentType> type_list;
    for (int i=0;i<5;++i)
        type_list.push_back(path.type_[i]);
    return type_list;
}

void ReedsSheppStateSpace::sample(double q0[3], double q1[3], double step_size, double &length, std::vector<std::vector<double> > &points)
{
    ReedsSheppPath path = reedsShepp(q0, q1);
    length = rho_ * path.length();

    for (double seg=0.0; seg<=length; seg+=step_size) {
        double qnew[3] = {};
        interpolate(q0, path, seg/rho_, qnew);
        std::vector<double> v(qnew, qnew + sizeof qnew / sizeof qnew[0]);
        points.push_back(v);
    }
    return;
}

void ReedsSheppStateSpace::interpolate(double q0[3], ReedsSheppPath &path, double seg, double s[3])
{

    if (seg < 0.0) seg = 0.0;
    if (seg > path.length()) seg = path.length();

    double phi, v;

    s[0] = s[1] = 0.0;
    s[2] = q0[2];

    for (unsigned int i=0; i<5 && seg>0; ++i)
    {
        if (path.length_[i]<0)
        {
            v = std::max(-seg, path.length_[i]);
            seg += v;
        }
        else
        {
            v = std::min(seg, path.length_[i]);
            seg -= v;
        }
        phi = s[2];
        switch(path.type_[i])
        {
            case RS_LEFT:
                s[0] += ( sin(phi+v) - sin(phi));
                s[1] += (-cos(phi+v) + cos(phi));
                s[2] = phi + v;
                break;
            case RS_RIGHT:
                s[0] += (-sin(phi-v) + sin(phi));
                s[1] += ( cos(phi-v) - cos(phi));
                s[2] = phi - v;
                break;
            case RS_STRAIGHT:
                s[0] += (v * cos(phi));
                s[1] += (v * sin(phi));
                break;
            case RS_NOP:
                break;
        }
    }

    s[0] = s[0] * rho_ + q0[0];
    s[1] = s[1] * rho_ + q0[1];
}
