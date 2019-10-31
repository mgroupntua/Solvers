using System;
using System.Collections.Generic;
using System.Linq;
using MGroup.LinearAlgebra.Interpolation;
using MGroup.LinearAlgebra.Matrices;
using MGroup.LinearAlgebra.Matrices.Builders;
using MGroup.LinearAlgebra.Vectors;
using MGroup.MSolve.Discretization;
using MGroup.MSolve.Discretization.DofOrdering;
using MGroup.MSolve.Geometry.Coordinates;
using MGroup.Solvers.DomainDecomposition.Overlapping.Schwarz.Additive.CoarseProblem;
using MGroup.Solvers.DomainDecomposition.Overlapping.Schwarz.Additive.Interfaces;

namespace MGroup.Solvers.DomainDecomposition.Overlapping.Schwarz.Additive
{
    public class ModelOverlappingDecomposition2D : IModelOverlappingDecomposition
    {
        private List<int>[] _connectivity;
        private readonly IAxisOverlappingDecomposition _ksiDecomposition;
        private readonly IAxisOverlappingDecomposition _hetaDecomposition;
        private readonly int _numberOfCpHeta;
        private List<IWeightedPoint> _patchControlPoints;
        private readonly IAsymmetricModel _model;
        private readonly ISubdomain _patch;
        private readonly ICoarseSpaceNodeSelection _coarseSpaceNodeSelection;
        private IMatrix _coarseSpaceInterpolation;
        private List<int> freeSubdomainDofs;

        public ModelOverlappingDecomposition2D(IAxisOverlappingDecomposition ksiDecomposition,
            IAxisOverlappingDecomposition hetaDecomposition, int numberOfCpHeta, List<IWeightedPoint> patchControlPoints,
            IAsymmetricModel model, ICoarseSpaceNodeSelection coarseSpaceNodeSelection, ISubdomain patch)
        {
            _ksiDecomposition = ksiDecomposition;
            _hetaDecomposition = hetaDecomposition;
            _numberOfCpHeta = numberOfCpHeta;
            _patchControlPoints = patchControlPoints;
            _coarseSpaceNodeSelection = coarseSpaceNodeSelection;
            _model = model;
            _patch = patch;
        }

        public int NumberOfSubdomains => _connectivity.GetLength(0);

        public IMatrix CoarseSpaceInterpolation => _coarseSpaceInterpolation;

        public void DecomposeMatrix()
        {
            _ksiDecomposition.DecomposeAxis();
            _hetaDecomposition.DecomposeAxis();

            CalculateLocalProblems();
            CreateCoarseSpaceInterpolation();
        }

        private void CreateCoarseSpaceInterpolation()
        {
            var degreKsi = _ksiDecomposition.Degree;
            var knotValueVectorKsi = _ksiDecomposition.KnotValueVector;
            var coarsePointsKsi = _ksiDecomposition.GetAxisCoarsePoints();

            var degreeHeta = _hetaDecomposition.Degree;
            var knotValueVectorHeta = _hetaDecomposition.KnotValueVector;
            var coarsePointsHeta = _hetaDecomposition.GetAxisCoarsePoints();
            var coarsePoints2D =
                (from ksi in coarsePointsKsi from heta in coarsePointsHeta select new NaturalPoint(ksi, heta)).ToList();

            var R0 = DokRowMajor.CreateEmpty(_patchControlPoints.Count * 2, coarsePoints2D.Count * 2);
            for (int i = 0; i < coarsePoints2D.Count; i++)
            {
                var NurbsValues = CalculateNURBS2D(degreKsi, degreeHeta, knotValueVectorKsi,
                    knotValueVectorHeta, coarsePoints2D[i], _patchControlPoints);

                for (int j = 0; j < NurbsValues.NumRows; j++)
                {
                    if (Math.Abs(NurbsValues[j, 0]) < 10e-9) continue;
                    R0[2 * j, 2 * i] = NurbsValues[j, 0];
                    R0[2 * j + 1, 2 * i + 1] = NurbsValues[j, 0];
                }
            }

            _coarseSpaceInterpolation = R0.BuildCsrMatrix(true).GetSubmatrix(freeSubdomainDofs.ToArray(),_coarseSpaceNodeSelection.GetFreeCoarseSpaceDofs());
        }

        private Matrix CalculateNURBS2D(int degreeKsi, int degreeHeta, IVector knotValueVectorKsi,
            IVector knotValueVectorHeta, NaturalPoint naturalPoint, List<IWeightedPoint> patchControlPoints)
        {
            var numberOfCPKsi = knotValueVectorKsi.Length - degreeKsi - 1;
            var numberOfCPHeta = knotValueVectorHeta.Length - degreeHeta - 1;

            BSPLines1D bsplinesKsi = new BSPLines1D(degreeKsi, knotValueVectorKsi, Vector.CreateFromArray(new double[] { naturalPoint.Xi }));
            BSPLines1D bsplinesHeta = new BSPLines1D(degreeHeta, knotValueVectorHeta, Vector.CreateFromArray(new double[] { naturalPoint.Eta }));
            bsplinesKsi.calculateBSPLines();
            bsplinesHeta.calculateBSPLines();

            int numberOfElementControlPoints = patchControlPoints.Count;

            var nurbsValues = Matrix.CreateZero(numberOfElementControlPoints, 1);

            double sumKsiHeta = 0;

            for (int k = 0; k < numberOfElementControlPoints; k++)
            {
                int indexKsi = patchControlPoints[k].ID / numberOfCPHeta;
                int indexHeta = patchControlPoints[k].ID % numberOfCPHeta;
                sumKsiHeta += bsplinesKsi.BSPLineValues[indexKsi,0] *
                              bsplinesHeta.BSPLineValues[indexHeta,0] *
                              patchControlPoints[k].WeightFactor;
            }

            for (int k = 0; k < numberOfElementControlPoints; k++)
            {
                int indexKsi = patchControlPoints[k].ID / numberOfCPHeta;
                int indexHeta = patchControlPoints[k].ID % numberOfCPHeta;

                nurbsValues[k, 0] =
                    bsplinesKsi.BSPLineValues[indexKsi,0] *
                    bsplinesHeta.BSPLineValues[indexHeta,0] *
                    patchControlPoints[k].WeightFactor / sumKsiHeta;
            }

            return nurbsValues;
        }

        public static int FindSpan(int numberOfBasisFunctions, int degree, double pointCoordinate, IVector knotValueVector)
        {
            if (pointCoordinate == knotValueVector[numberOfBasisFunctions + 1]) return numberOfBasisFunctions;
            int minimum = degree;
            int maximum = numberOfBasisFunctions + 1;
            int mid = (minimum + maximum) / 2;
            while (pointCoordinate < knotValueVector[mid] || pointCoordinate >= knotValueVector[mid + 1])
            {
                if (pointCoordinate < knotValueVector[mid])
                    maximum = mid;
                else
                    minimum = mid;
                mid = (minimum + maximum) / 2;
            }

            return mid;
        }

        public static Vector BasisFunctions(int spanId, double pointCoordinate, int degree, IVector knotValueVector)
        {
            var basisFunctions = Vector.CreateZero(degree + 1);
            var left = Vector.CreateZero(degree + 1);
            var right = Vector.CreateZero(degree + 1);
            basisFunctions[0] = 1;
            for (int j = 1; j <= degree; j++)
            {
                left[j] = pointCoordinate - knotValueVector[spanId + 1 - j];
                right[j] = knotValueVector[spanId + j] - pointCoordinate;
                var saved = 0.0;
                for (int r = 0; r < j; r++)
                {
                    var temp = basisFunctions[r] / (right[r + 1] + left[j - r]);
                    basisFunctions[r] = saved + right[r + 1] * temp;
                    saved = left[j - r] * temp;
                }

                basisFunctions[j] = saved;
            }

            return basisFunctions;
        }

		// TODO: ** Abstract the StructuralDOF references **
		private void CalculateLocalProblems()
        {
            ////TODO: add dofOrderer to ignored constrainedDofs of each subdomain
            //var numberOfSubdomainKsi = _ksiDecomposition.NumberOfAxisSubdomains;
            //var numberOfSubdomainHeta = _hetaDecomposition.NumberOfAxisSubdomains;
            //_connectivity = new List<int>[numberOfSubdomainKsi * numberOfSubdomainHeta];
            
            //var indexSubdomain = -1;
            //for (int i = 0; i < numberOfSubdomainKsi; i++)
            //{
            //    var subdomainKsiConnectivity = _ksiDecomposition.GetAxisIndicesOfSubdomain(i);
            //    for (int j = 0; j < numberOfSubdomainHeta; j++)
            //    {
            //        var subdomainHetaConnectivity = _hetaDecomposition.GetAxisIndicesOfSubdomain(j);
            //        indexSubdomain++;
            //        _connectivity[indexSubdomain] =
            //            new List<int>();

            //        freeSubdomainDofs=new List<int>();
            //        foreach (var controlPoint in _patchControlPoints)
            //        {
            //            if (_model.GlobalColDofOrdering.GlobalFreeDofs.Contains(controlPoint, StructuralDof.TranslationX))
            //                freeSubdomainDofs.Add(2*controlPoint.ID);

            //            if (_model.GlobalColDofOrdering.GlobalFreeDofs.Contains(controlPoint, StructuralDof.TranslationX))
            //                freeSubdomainDofs.Add(2 * controlPoint.ID+1);
            //        }

            //        foreach (var indexKsi in subdomainKsiConnectivity)
            //        {
            //            foreach (var indexHeta in subdomainHetaConnectivity)
            //            {
            //                var indexCP = indexKsi * _numberOfCpHeta + indexHeta;
            //                var node = _patchControlPoints[indexCP] as INode;
            //                if (_model.GlobalColDofOrdering.GlobalFreeDofs.Contains(node, StructuralDof.TranslationX))
            //                    _connectivity[indexSubdomain].Add(_model.GlobalColDofOrdering.GlobalFreeDofs[node, StructuralDof.TranslationX]);
            //                if (_model.GlobalColDofOrdering.GlobalFreeDofs.Contains(node, StructuralDof.TranslationY))
            //                    _connectivity[indexSubdomain].Add(_model.GlobalColDofOrdering.GlobalFreeDofs[node, StructuralDof.TranslationY]);

            //            }
            //        }
            //    }
            //}
        }

        public int[] GetConnectivityOfSubdomain(int indexSubdomain) => _connectivity[indexSubdomain].ToArray();

    }
}
