//namespace MGroup.MachineLearning.Utilities
//{
//	using System;
//	using System.Collections.Generic;
//	using System.Diagnostics;
//	using System.Linq;
//	using System.Text;

//	// TODO: Allow caller to choose if the test set and the validation set will be min (Ceiling()), max (Floor()) or best fit (Round())
//	// TODO: Allow caller to specify which array dimension will be taken as the samples. E.g. now in a 2D array, each row is a sample
//	// TODO: Use a mutable (builder) class (with chaining properties) to create an immutable splitting rule which can be used
//	//		 to split multiple datasets.
//	// TODO: Test this extensively
//	public class DatasetSplitter
//	{
//		private double _minTestSetPercentage = 0.20;
//		private double _minValidationSetPercentage = 0.0;
//		private DataSubsetType[] _setOrder = { DataSubsetType.Training, DataSubsetType.Test, DataSubsetType.Validation };
//		private bool _orderIsRandom;
//		private int? _randomSeed;

//		private int _totalDatasetSize = -1;
//		private int _trainingSetStart = -1;
//		private int _trainingSetEnd = -1; // exclusive
//		private int _testSetStart = -1;
//		private int _testSetEnd = -1; // exclusive
//		private int _validationSetStart = -1;
//		private int _validationSetEnd = -1; // exclusive

//		/// <summary>
//		/// Defaults to 20%. If not set to exactly 0%, at least one sample will be added to it.
//		/// </summary>
//		public double MinTestSetPercentage
//		{
//			get => _minTestSetPercentage;
//			set
//			{
//				if (value < 0.0 || value >= 1.0)
//				{
//					throw new ArgumentException("Test set percentage must be in the range [0.0, 1.0)");
//				}

//				_minTestSetPercentage = value;
//			}
//		}

//		/// <summary>
//		/// Defaults to 0%. If not set to exactly 0%, at least one sample will be added to it.
//		/// </summary>
//		public double MinValidationSetPercentage
//		{
//			get => _minValidationSetPercentage;
//			set
//			{
//				if (value < 0.0 || value >= 1.0)
//				{
//					throw new ArgumentException("Validation set percentage must be in the range [0.0, 1.0)");
//				}

//				_minValidationSetPercentage = value;
//			}
//		}

//		/// <summary>
//		/// Defines in which order to distribute samples of the full data set to the training, test, validation sets.
//		/// The caller must specify the first and second of training/test/validation sets, but the last one will be inferred.
//		/// Overwrites the results of the previous calls to <see cref="SetOrderToContiguous(DataSubsetType, DataSubsetType)"/>
//		/// and <see cref="SetOrderToRandom(int?)"/>. By default, the distribution order is not random.
//		/// The default contiguous order is training set, then test set, then validation set.
//		/// </summary>
//		/// <param name="first">Must not be the same as <paramref name="second"/>.</param>
//		/// <param name="second">Must not be the same as <paramref name="first"/>.</param>
//		public void SetOrderToContiguous(DataSubsetType first, DataSubsetType second)
//		{
//			if (first == second)
//			{
//				throw new ArgumentException("The first and second set types must not be the same.");
//			}

//			_setOrder[0] = first;
//			_setOrder[1] = second;

//			// Find last one
//			var setTypes = new HashSet<DataSubsetType>();
//			setTypes.UnionWith((DataSubsetType[])Enum.GetValues(typeof(DataSubsetType)));
//			setTypes.Remove(first);
//			setTypes.Remove(second);
//			Debug.Assert(setTypes.Count == 1, "Something went wrong, but the caller is innocent.");
//			_setOrder[2] = setTypes.First();

//			// Turn off randomization
//			_orderIsRandom = false;
//			_randomSeed = null;

//			ClearSplittingRules();
//		}

//		/// <summary>
//		/// Randomly distributes the samples of the full data set to the training, test, validation sets.
//		/// Overwrites the results of the previous calls to <see cref="SetOrderToRandom(int?)"/> and
//		/// <see cref="SetOrderToContiguous(SetType, SetType)"/>. By default, the distribution order is not random.
//		/// </summary>
//		/// <param name="seed">
//		/// The seed to control the random distribution of samples. If no value is provided, the default time-dependent seed 
//		/// of C# will be used.
//		/// </param>
//		public void SetOrderToRandom(int? seed = null)
//		{
//			throw new NotImplementedException();
//			_orderIsRandom = true;
//			_randomSeed = seed;

//			// Clear contiguous order
//			_setOrder = new DataSubsetType[] { DataSubsetType.Training, DataSubsetType.Test, DataSubsetType.Validation };

//			ClearSplittingRules();
//		}

//		/// <summary>
//		/// Determines how to split a dataset consisting of <paramref name="totalDatasetSize"/> samples to the training, test and
//		/// validation sets. All subsequent calls to <see cref="SplitDataset{T}(T[])"/>, <see cref="SplitDataset{T}(T[,])"/>, etc.
//		/// will use the rules created by this method, thus the provided datasets must have <paramref name="totalDatasetSize"/> 
//		/// samples. Calling <see cref="SetOrderToContiguous(SetType, SetType)"/> or <see cref="SetOrderToRandom(int?)"/> will
//		/// delete any splitting rules created by this method, therefore it must be called again.
//		/// </summary>
//		public void SetupSplittingRules(int totalDatasetSize)
//		{
//			(int trainingSetSize, int testSetSize, int validationSetSize) = FindSetSizes(totalDatasetSize);
//			_totalDatasetSize = totalDatasetSize;

//			int offset = 0;
//			for (int section = 0; section < 3; ++section)
//			{
//				if (_setOrder[section] == DataSubsetType.Training)
//				{
//					_trainingSetStart = offset;
//					_trainingSetEnd = offset + trainingSetSize;
//					offset += trainingSetSize;
//				}
//				else if (_setOrder[section] == DataSubsetType.Test)
//				{
//					if (testSetSize > 0)
//					{
//						_testSetStart = offset;
//						_testSetEnd = offset + testSetSize;
//						offset += testSetSize;
//					}
//				}
//				else
//				{
//					if (validationSetSize > 0)
//					{
//						_validationSetStart = offset;
//						_validationSetEnd = offset + validationSetSize;
//						offset += validationSetSize;
//					}
//				}
//			}
//		}

//		public (T[] trainingSet, T[] testSet, T[] validationSet) SplitDataset<T>(T[] totalDataset)
//		{
//			CheckSplittingRules(totalDataset.Length);
//			T[] trainingSet = totalDataset.Slice(_trainingSetStart, _trainingSetEnd);

//			T[] testSet;
//			if (_testSetStart > 0)
//			{
//				testSet = totalDataset.Slice(_testSetStart, _testSetEnd);
//			}
//			else
//			{
//				testSet = Array.Empty<T>();
//			}

//			T[] validationSet;
//			if (_testSetStart > 0)
//			{
//				validationSet = totalDataset.Slice(_validationSetStart, _validationSetEnd);
//			}
//			else
//			{
//				validationSet = Array.Empty<T>();
//			}

//			return (trainingSet, testSet, validationSet);
//		}

//		public (T[,] trainingSet, T[,] testSet, T[,] validationSet) SplitDataset<T>(T[,] totalDataset)
//		{
//			CheckSplittingRules(totalDataset.GetLength(0));
//			T[,] trainingSet = totalDataset.Slice((_trainingSetStart, _trainingSetEnd), null);

//			T[,] testSet;
//			if (_testSetStart > 0)
//			{
//				testSet = totalDataset.Slice((_testSetStart, _testSetEnd), null);
//			}
//			else
//			{
//				testSet = new T[0, 0];
//			}

//			T[,] validationSet;
//			if (_testSetStart > 0)
//			{
//				validationSet = totalDataset.Slice((_validationSetStart, _validationSetEnd), null);
//			}
//			else
//			{
//				validationSet = new T[0, 0];
//			}

//			return (trainingSet, testSet, validationSet);
//		}

//		public (T[,,] trainingSet, T[,,] testSet, T[,,] validationSet) SplitDataset<T>(T[,,] totalDataset)
//		{
//			CheckSplittingRules(totalDataset.GetLength(0));
//			T[,,] trainingSet = totalDataset.Slice((_trainingSetStart, _trainingSetEnd), null, null);

//			T[,,] testSet;
//			if (_testSetStart > 0)
//			{
//				testSet = totalDataset.Slice((_testSetStart, _testSetEnd), null, null);
//			}
//			else
//			{
//				testSet = new T[0, 0, 0];
//			}

//			T[,,] validationSet;
//			if (_testSetStart > 0)
//			{
//				validationSet = totalDataset.Slice((_validationSetStart, _validationSetEnd), null, null);
//			}
//			else
//			{
//				validationSet = new T[0, 0, 0];
//			}

//			return (trainingSet, testSet, validationSet);
//		}

//		public (T[,,,] trainingSet, T[,,,] testSet, T[,,,] validationSet) SplitDataset<T>(T[,,,] totalDataset)
//		{
//			CheckSplittingRules(totalDataset.GetLength(0));
//			T[,,,] trainingSet = totalDataset.Slice((_trainingSetStart, _trainingSetEnd), null, null, null);

//			T[,,,] testSet;
//			if (_testSetStart > 0)
//			{
//				testSet = totalDataset.Slice((_testSetStart, _testSetEnd), null, null, null);
//			}
//			else
//			{
//				testSet = new T[0, 0, 0, 0];
//			}

//			T[,,,] validationSet;
//			if (_testSetStart > 0)
//			{
//				validationSet = totalDataset.Slice((_validationSetStart, _validationSetEnd), null, null, null);
//			}
//			else
//			{
//				validationSet = new T[0, 0, 0, 0];
//			}

//			return (trainingSet, testSet, validationSet);
//		}

//		public (T[,,,,] trainingSet, T[,,,,] testSet, T[,,,,] validationSet) SplitDataset<T>(T[,,,,] totalDataset)
//		{
//			CheckSplittingRules(totalDataset.GetLength(0));
//			T[,,,,] trainingSet = totalDataset.Slice((_trainingSetStart, _trainingSetEnd), null, null, null, null);

//			T[,,,,] testSet;
//			if (_testSetStart > 0)
//			{
//				testSet = totalDataset.Slice((_testSetStart, _testSetEnd), null, null, null, null);
//			}
//			else
//			{
//				testSet = new T[0, 0, 0, 0, 0];
//			}

//			T[,,,,] validationSet;
//			if (_testSetStart > 0)
//			{
//				validationSet = totalDataset.Slice((_validationSetStart, _validationSetEnd), null, null, null, null);
//			}
//			else
//			{
//				validationSet = new T[0, 0, 0, 0, 0];
//			}

//			return (trainingSet, testSet, validationSet);
//		}

//		private void CheckSplittingRules(int totalDatasetSize)
//		{
//			if (_totalDatasetSize == -1)
//			{
//				throw new InvalidOperationException("The splitting rules must be created first.");
//			}

//			if (totalDatasetSize != _totalDatasetSize)
//			{
//				throw new ArgumentException($"The size of the provided dataset must be {_totalDatasetSize}, " +
//					$"but was {totalDatasetSize}");
//			}
//		}

//		private void ClearSplittingRules()
//		{
//			_totalDatasetSize = -1;
//			_trainingSetStart = -1;
//			_trainingSetEnd = -1;
//			_testSetStart = -1;
//			_testSetEnd = -1;
//			_validationSetStart = -1;
//			_validationSetEnd = -1;
//		}

//		private (int trainingSetSize, int testSetSize, int validationSetSize) FindSetSizes(int totalDatasetSize)
//		{
//			if (totalDatasetSize < 2)
//			{
//				throw new ArgumentException("The provided dataset must have at least 2 samples.");
//			}

//			int testSetSize = (int)Math.Ceiling(_minTestSetPercentage * totalDatasetSize);
//			if (_minTestSetPercentage == 0.0)
//			{
//				testSetSize = 0;
//			}
//			else if (testSetSize == 0)
//			{
//				testSetSize = 1;
//			}

//			int validationSetSize = (int)Math.Ceiling(_minValidationSetPercentage * totalDatasetSize);
//			if (_minValidationSetPercentage == 0.0)
//			{
//				validationSetSize = 0;
//			}
//			else if (validationSetSize == 0)
//			{
//				validationSetSize = 1;
//			}

//			int trainingSetSize = totalDatasetSize - testSetSize - validationSetSize;
//			if (trainingSetSize < 1)
//			{
//				throw new ArgumentException(
//					$"The total dataset size ({totalDatasetSize}) is too small and nothing is left for the training set " +
//					$"after the test set ({testSetSize}) and validation set ({validationSetSize}) are created.");
//			}

//			return (trainingSetSize, testSetSize, validationSetSize);
//		}
//	}
//}
