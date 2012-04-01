using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;

namespace ArdupilotMega
{
    class PIDTunning
    {

        public static void twiddle(double[] initialgains, Func<double[],double> run, double tol = 0.001)
        {
            int n_params = 3;
            double err= 0;
            double[] dparams = initialgains; //{1.0f,1.0f,1.0f};
            double[] paramss = {0.0f,0.0f,0.0f};
            double best_error = run(paramss);
            int n = 0;

            while (dparams.Sum() > tol) {
                for (int i = 0; i < n_params; i++){
                    paramss[i] += dparams[i];
                    err = run(paramss);
                    if (err < best_error){
                        best_error = err;
                        dparams[i] *= 1.1;
                    }
                    else {
                        paramss[i] -= 2.0 * dparams[i];
                        err = run(paramss);
                        if (err < best_error){
                            best_error = err;
                            dparams[i] *= 1.1;
                        }
                        else {
                            paramss[i] += dparams[i];
                            dparams[i] *= 0.9;
                        }
                    }
                    n += 1;
                    Console.WriteLine("Twiddle #" + n + " " + paramss.ToString() + " -> " + best_error);
                }
            }
        }
    }
}
