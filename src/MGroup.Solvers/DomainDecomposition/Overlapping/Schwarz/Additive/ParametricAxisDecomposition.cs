using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using MGroup.LinearAlgebra.Vectors;
using MGroup.Solvers.DomainDecomposition.Overlapping.Schwarz.Additive.Interfaces;

namespace MGroup.Solvers.DomainDecomposition.Overlapping.Schwarz.Additive
{
    public class ParametricAxisDecomposition:IAxisOverlappingDecomposition
    {
        private List<double> _decompositionIndices;
        private int[][] _subdomainIndices;
        private IVector _knotValueVector;
        private int _numberOfSubdomains;
        private int _degree;
        private double[] _coarsePoints;
        private double[] _coarseKnotVector;

        public ParametricAxisDecomposition(IVector knotValueVector,int degree, int numberOfSubdomains)
        {
            _knotValueVector = knotValueVector;
            _numberOfSubdomains = numberOfSubdomains;
            _degree = degree;
        }

        public int NumberOfAxisSubdomains => _subdomainIndices.GetLength(0);

        public int Degree => _degree;

        public IVector KnotValueVector => _knotValueVector;

        public void DecomposeAxis()
        {
            var numberOfShapeFunctions = _knotValueVector.Length - _degree - 1;
            var shapeFunctionSupports = new List<FunctionSupport>();
            var indexFunction = 0;
            for (int i = 0; i < numberOfShapeFunctions; i++)
            {
                shapeFunctionSupports.Add(new FunctionSupport
                {
                    ID = indexFunction++,
                    Start = _knotValueVector[i],
                    End = _knotValueVector[i + _degree + 1]
                });
            }

            var knots = _knotValueVector.CopyToArray().Distinct().ToList();
            var axisElements = knots.Count() - 1;
            var subdomainElements = (axisElements % _numberOfSubdomains == 0)
                ? axisElements / _numberOfSubdomains
                : axisElements / _numberOfSubdomains + 1;

            _decompositionIndices = new List<double>();
            for (int i = 0; i < knots.Count() - 1; i += subdomainElements)
                _decompositionIndices.Add(knots[i]);
            _decompositionIndices.Add(knots[knots.Count - 1]);

            var coarseKnotVector = new List<double>();
            for (int i = 0; i < _degree; i++)
                coarseKnotVector.Add(knots[0]);
            coarseKnotVector.AddRange(_decompositionIndices);
            for (int i = 0; i < _degree; i++)
                coarseKnotVector.Add(knots[knots.Count-1]);

            _coarseKnotVector = coarseKnotVector.ToArray();

            var numberOfCoarsePoints = coarseKnotVector.Count - _degree - 1;
            _coarsePoints=new double[numberOfCoarsePoints];
            for (int i = 0; i < numberOfCoarsePoints; i++)
            {
                var sum = 0.0;
                for (int j = i+1; j <= i+_degree; j++)
                {
                    sum += coarseKnotVector[j];
                }
                _coarsePoints[i]=sum/_degree;
            }
            

            var shapeFunctionsIds = new List<int>();
            shapeFunctionsIds.Add(0);
            for (int i = 1; i < _decompositionIndices.Count - 1; i++)
            {
                var index = _decompositionIndices[i];
                var functions = shapeFunctionSupports.Where(s => s.Start < index && s.End > index).ToArray();
                var j = functions.Count() % 2 != 0 ? functions.Count() / 2 : functions.Count() / 2 - 1;
                var id = functions[j].ID;
                shapeFunctionsIds.Add(id);
            }
            shapeFunctionsIds.Add(numberOfShapeFunctions);

            _subdomainIndices = new int[shapeFunctionsIds.Count - 1][];
            for (int i = 0; i < shapeFunctionsIds.Count - 1; i++)
            {
                var indexCount = (shapeFunctionsIds[i] == 0)
                    ? shapeFunctionsIds[i + 1] - shapeFunctionsIds[i] + 1
                    : shapeFunctionsIds[i + 1] - shapeFunctionsIds[i];
                _subdomainIndices[i] = Enumerable.Range(shapeFunctionsIds[i], indexCount).ToArray();
            }
        }

        public double[] GetAxisCoarseKnotVector() => _decompositionIndices.ToArray();

        public double[] GetAxisCoarsePoints() => _coarsePoints;

        public int[] GetAxisIndicesOfSubdomain(int indexAxisSubdomain) => _subdomainIndices[indexAxisSubdomain];
    }

    public class FunctionSupport
    {
        public int ID { get; set; }
        public double Start { get; set; }
        public double End { get; set; }
    }
}
