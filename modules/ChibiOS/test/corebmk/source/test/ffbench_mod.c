/*
    ChibiOS - Copyright (C) 2006-2026 Giovanni Di Sirio.

    Licensed under the Apache License, Version 2.0 (the "License");
    you may not use this file except in compliance with the License.
    You may obtain a copy of the License at

        http://www.apache.org/licenses/LICENSE-2.0

    Unless required by applicable law or agreed to in writing, software
    distributed under the License is distributed on an "AS IS" BASIS,
    WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
    See the License for the specific language governing permissions and
    limitations under the License.
*/
/*
    This module is based on the work of John Walker (April of 1989) and
    merely adapted to work in ChibiOS. The author has not specified
    additional license terms so this is released using the most permissive
    license used in ChibiOS. The license covers the changes only, not the
    original work.
 */

#include <math.h>

#define SWAP(a, b) tempr=(a); (a)=(b); (b)=tempr

void fourn_float(float data[], int nn[], const int ndim, const int isign) {
  int i1, i2, i3;
  int i2rev, i3rev, ip1, ip2, ip3, ifp1, ifp2;
  int ibit, idim, k1, k2, n, nprev, nrem, ntot;
  float tempi, tempr;
  float theta, wi, wpi, wpr, wr, wtemp;

  ntot = 1;
  for (idim = 1; idim <= ndim; idim++) {
    ntot *= nn[idim];
  }
  nprev = 1;
  for (idim = ndim; idim >= 1; idim--) {
    n = nn[idim];
    nrem = ntot / (n * nprev);
    ip1 = nprev << 1;
    ip2 = ip1 * n;
    ip3 = ip2 * nrem;
    i2rev = 1;
    for (i2 = 1; i2 <= ip2; i2 += ip1) {
      if (i2 < i2rev) {
        for (i1 = i2; i1 <= i2 + ip1 - 2; i1 += 2) {
          for (i3 = i1; i3 <= ip3; i3 += ip2) {
            i3rev = i2rev + i3 - i2;
            SWAP(data[i3], data[i3rev]);
            SWAP(data[i3 + 1], data[i3rev + 1]);
          }
        }
      }
      ibit = ip2 >> 1;
      while (ibit >= ip1 && i2rev > ibit) {
        i2rev -= ibit;
        ibit >>= 1;
      }
      i2rev += ibit;
    }
    ifp1 = ip1;
    while (ifp1 < ip2) {
      ifp2 = ifp1 << 1;
      theta = (float)isign * 6.28318530717959f / (ifp2 / ip1);
      wtemp = sinf(0.5f * theta);
      wpr = -2.0f * wtemp * wtemp;
      wpi = sinf(theta);
      wr = 1.0f;
      wi = 0.0f;
      for (i3 = 1; i3 <= ifp1; i3 += ip1) {
        for (i1 = i3; i1 <= i3 + ip1 - 2; i1 += 2) {
          for (i2 = i1; i2 <= ip3; i2 += ifp2) {
            k1 = i2;
            k2 = k1 + ifp1;
            tempr = wr * data[k2] - wi * data[k2 + 1];
            tempi = wr * data[k2 + 1] + wi * data[k2];
            data[k2] = data[k1] - tempr;
            data[k2 + 1] = data[k1 + 1] - tempi;
            data[k1] += tempr;
            data[k1 + 1] += tempi;
          }
        }
        wr = (wtemp = wr) * wpr - wi * wpi + wr;
        wi = wi * wpr + wtemp * wpi + wi;
      }
      ifp1 = ifp2;
    }
    nprev *= n;
  }
}

void fourn_double(double data[], int nn[], const int ndim, const int isign) {
  int i1, i2, i3;
  int i2rev, i3rev, ip1, ip2, ip3, ifp1, ifp2;
  int ibit, idim, k1, k2, n, nprev, nrem, ntot;
  double tempi, tempr;
  double theta, wi, wpi, wpr, wr, wtemp;

  ntot = 1;
  for (idim = 1; idim <= ndim; idim++) {
    ntot *= nn[idim];
  }
  nprev = 1;
  for (idim = ndim; idim >= 1; idim--) {
    n = nn[idim];
    nrem = ntot / (n * nprev);
    ip1 = nprev << 1;
    ip2 = ip1 * n;
    ip3 = ip2 * nrem;
    i2rev = 1;
    for (i2 = 1; i2 <= ip2; i2 += ip1) {
      if (i2 < i2rev) {
        for (i1 = i2; i1 <= i2 + ip1 - 2; i1 += 2) {
          for (i3 = i1; i3 <= ip3; i3 += ip2) {
            i3rev = i2rev + i3 - i2;
            SWAP(data[i3], data[i3rev]);
            SWAP(data[i3 + 1], data[i3rev + 1]);
          }
        }
      }
      ibit = ip2 >> 1;
      while (ibit >= ip1 && i2rev > ibit) {
        i2rev -= ibit;
        ibit >>= 1;
      }
      i2rev += ibit;
    }
    ifp1 = ip1;
    while (ifp1 < ip2) {
      ifp2 = ifp1 << 1;
      theta = isign * 6.28318530717959 / (ifp2 / ip1);
      wtemp = sin(0.5 * theta);
      wpr = -2.0 * wtemp * wtemp;
      wpi = sin(theta);
      wr = 1.0;
      wi = 0.0;
      for (i3 = 1; i3 <= ifp1; i3 += ip1) {
        for (i1 = i3; i1 <= i3 + ip1 - 2; i1 += 2) {
          for (i2 = i1; i2 <= ip3; i2 += ifp2) {
            k1 = i2;
            k2 = k1 + ifp1;
            tempr = wr * data[k2] - wi * data[k2 + 1];
            tempi = wr * data[k2 + 1] + wi * data[k2];
            data[k2] = data[k1] - tempr;
            data[k2 + 1] = data[k1 + 1] - tempi;
            data[k1] += tempr;
            data[k1 + 1] += tempi;
          }
        }
        wr = (wtemp = wr) * wpr - wi * wpi + wr;
        wi = wi * wpr + wtemp * wpi + wi;
      }
      ifp1 = ifp2;
    }
    nprev *= n;
  }
}
