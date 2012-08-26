using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Collections;
using GMap.NET;
using AStar;

namespace ArdupilotMega.Utilities
{
    class PathFind
    {
        public static List<PointLatLng> FindPath(PointLatLng start, List<GCSViews.FlightPlanner.linelatlng> list)
        {

            Graph graph = new Graph();

            Node home = new Node("Home", null, start.Lat, start.Lng);

            Node destination = new Node("End", null, start.Lat, start.Lng);

            graph.AddNode(home);
            graph.AddNode(destination);

            int a = 0;

            foreach (GCSViews.FlightPlanner.linelatlng line in list)
            {
                Node n1 = new Node(a.ToString(), null, line.p1.Lat, line.p1.Lng);
                a++;
                Node n2 = new Node(a.ToString(), null, line.p2.Lat, line.p2.Lng);
                a++;

                graph.AddNode(n1);
                graph.AddNode(n2);

                // add the home to point initial refrence - 1 way
                graph.AddDirectedEdge(home, n1, (int)GetDistance(start,line.p1));
                graph.AddDirectedEdge(home, n2, (int)GetDistance(start, line.p2));

                // add points to destination
                graph.AddDirectedEdge(n1, destination, (int)GetDistance(start, line.p1));
                graph.AddDirectedEdge(n2, destination, (int)GetDistance(start, line.p2));

                // 0 cost as it is a line
                graph.AddUndirectedEdge(n1, n2, 0);

                foreach (GCSViews.FlightPlanner.linelatlng line2 in list)
                {
                    Node n11 = new Node(a.ToString(), null, line2.p1.Lat, line2.p1.Lng);
                    a++;
                    Node n21 = new Node(a.ToString(), null, line2.p2.Lat, line2.p2.Lng);
                    a++;

                    if (line.p1 == line2.p1 && line.p2 == line2.p2)
                        continue;

                    graph.AddNode(n11);
                    graph.AddNode(n21);

                    graph.AddUndirectedEdge(n1, n11, GetDistance(line.p1, line2.p1));
                    graph.AddUndirectedEdge(n2, n11, GetDistance(line.p2, line2.p1));
                    graph.AddUndirectedEdge(n1, n21, GetDistance(line.p1, line2.p2));
                    graph.AddUndirectedEdge(n2, n21, GetDistance(line.p2, line2.p2));

                    // 0 cost
                    graph.AddUndirectedEdge(n11, n21, 0);
                }
            }

            // Function which tells us the exact distance between two neighbours.
            Func<Node, Node, double> distance = new Func<Node, Node, double>(PathFind.distance);

            // Estimation/Heuristic function (Haversine distance)
            // It tells us the estimated distance between the last node on a proposed path and the destination node.
            Func<Node, double> haversineEstimation =
                n => Haversine.Distance(n, destination, DistanceType.km);

           // foreach (GCSViews.FlightPlanner.linelatlng line in list)
           // {

                Path<Node> shortestPath = AStar.AStar.FindPath(home, destination, distance, haversineEstimation);

           // }

            List<PointLatLng> shortest = new List<PointLatLng>();
            
            foreach (Path<Node> path in shortestPath)
            {
                if (path.PreviousSteps != null)
                {
                    shortest.Add(new PointLatLng(path.PreviousSteps.LastStep.Latitude, path.PreviousSteps.LastStep.Longitude));
                }
            }
            
            return shortest;
        }

        public static double distance(Node node1, Node node2)
        {
            return node1.Neighbors.Cast<EdgeToNeighbor>().Single(
                 etn => etn.Neighbor.Key == node2.Key).Cost;
        }


        static PointLatLng findClosestPoint(PointLatLng start, GCSViews.FlightPlanner.linelatlng line)
        {
            List<PointLatLng> list = new List<PointLatLng>();
            list.Add(line.p1);
            list.Add(line.p2);

            PointLatLng closest = findClosestPoint(start, list);

            return closest;
        }

        static PointLatLng findClosestPoint(PointLatLng start, List<PointLatLng> list)
        {
            PointLatLng answer = PointLatLng.Zero;
            double currentbest = double.MaxValue;

            foreach (PointLatLng pnt in list)
            {
                double dist1 = GetDistance(start, pnt);

                if (dist1 < currentbest)
                {
                    answer = pnt;
                    currentbest = dist1;
                }
            }

            return answer;
        }

        static GCSViews.FlightPlanner.linelatlng findClosestLine(PointLatLng start, List<GCSViews.FlightPlanner.linelatlng> list)
        {
            GCSViews.FlightPlanner.linelatlng answer = list[0];
            double shortest = double.MaxValue;

            foreach (GCSViews.FlightPlanner.linelatlng line in list)
            {
                double ans1 = GetDistance(start, line.p1);
                double ans2 = GetDistance(start, line.p2);
                PointLatLng shorterpnt = ans1 < ans2 ? line.p1 : line.p2;

                if (shortest > GetDistance(start, shorterpnt))
                {
                    answer = line;
                    shortest = GetDistance(start, shorterpnt);
                }
            }

            return answer;
        }

        /// <summary>
        /// Calc Distance in M
        /// </summary>
        /// <param name="p2"></param>
        /// <returns>Distance in M</returns>
        public static double GetDistance(PointLatLng p1, PointLatLng p2)
        {
            double d = p1.Lat * 0.017453292519943295;
            double num2 = p1.Lng * 0.017453292519943295;
            double num3 = p2.Lat * 0.017453292519943295;
            double num4 = p2.Lng * 0.017453292519943295;
            double num5 = num4 - num2;
            double num6 = num3 - d;
            double num7 = Math.Pow(Math.Sin(num6 / 2.0), 2.0) + ((Math.Cos(d) * Math.Cos(num3)) * Math.Pow(Math.Sin(num5 / 2.0), 2.0));
            double num8 = 2.0 * Math.Atan2(Math.Sqrt(num7), Math.Sqrt(1.0 - num7));
            return (6371 * num8); // M
        }
    }
}
