using System;
using System.Collections.Generic;
using System.Text;
using MGroup.LinearAlgebra.Matrices;
using MGroup.LinearAlgebra.Matrices.Builders;
using MGroup.LinearAlgebra.Triangulation;
using MGroup.Solvers.DomainDecomposition.Overlapping.Schwarz.Additive.Interfaces;

namespace MGroup.Solvers.DomainDecomposition.Overlapping.Schwarz.Additive
{
    public class LocalProblemsFactory
    {
        private Matrix[] inverseMatrices;
        private IModelOverlappingDecomposition _modelOverlappingDecomposition;
        private int _matrixOrder;

        public void GenerateMatrices(IMatrixView matrix, IModelOverlappingDecomposition modelOverlappingDecomposition)
        {
            _modelOverlappingDecomposition = modelOverlappingDecomposition;
            _matrixOrder = matrix.NumRows;
            inverseMatrices = new Matrix[modelOverlappingDecomposition.NumberOfSubdomains];

            for (int i = 0; i < modelOverlappingDecomposition.NumberOfSubdomains; i++)
            {
                var subdomainConnectivity = modelOverlappingDecomposition.GetConnectivityOfSubdomain(i);
                var submatrix=matrix.GetSubmatrix(subdomainConnectivity, subdomainConnectivity).CopyToFullMatrix();
                inverseMatrices[i]= submatrix.Invert();
            }
        }

        public Matrix RetrievePreconditionerContribution()
        {
            var localProblemsContribution =Matrix.CreateZero(_matrixOrder, _matrixOrder);
            for (int i = 0; i < _modelOverlappingDecomposition.NumberOfSubdomains; i++)
            {
                var subdomainConnectivity = _modelOverlappingDecomposition.GetConnectivityOfSubdomain(i);
                var inverseMatrix = inverseMatrices[i];
                for (int j = 0; j < subdomainConnectivity.Length; j++)
                {
                    for (int k = 0; k < subdomainConnectivity.Length; k++)
                    {
                        localProblemsContribution[subdomainConnectivity[j], subdomainConnectivity[k]] +=
                            inverseMatrix[j, k];
                    }
                }
            }

            return localProblemsContribution;
        }
    }
}