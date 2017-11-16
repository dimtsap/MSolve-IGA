using ISAAR.MSolve.IGA.Interfaces;
using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using ISAAR.MSolve.Numerical.LinearAlgebra.Interfaces;
using ISAAR.MSolve.IGA.Entities;

namespace ISAAR.MSolve.IGA.Problems.Structural.Providers
{
    public class ElementStructuralStiffnessProvider : IElementMatrixProvider
    {
        #region IElementMatrixProvider Members
        public IMatrix2D Matrix(Element element)
        {
            return element.ElementType.StiffnessMatrix(element);
        }
        #endregion
    }
}
