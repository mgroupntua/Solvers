namespace MGroup.Solvers.DDM.MLExtensions.PythonInterop
{
	using System;
	using System.Collections.Generic;
	using System.Linq;
	using System.Reflection;
	using System.Text;

	using Newtonsoft.Json;

	public abstract class IoParamsManagerBase<T> where T : new()
	{
		protected readonly Type paramsType = typeof(T);
		protected readonly IReadOnlyList<PropertyInfo> arrayProps;
		//protected readonly IReadOnlyList<PropertyInfo> serializableProps;
		protected readonly JsonSerializer serializer;

		public IoParamsManagerBase()
		{
			List<PropertyInfo> arrayProps = [];
			List<PropertyInfo> serializableProps = [];

			var allProperties = paramsType.GetProperties(BindingFlags.Public | BindingFlags.Instance);
			foreach (var prop in allProperties)
			{
				if (prop.PropertyType.IsArray)
				{
					arrayProps.Add(prop);
				}
				//else
				//{
				//	serializableProps.Add(prop);
				//}
			}

			this.arrayProps = arrayProps;
			//this.serializableProps = serializableProps;

			var serializerSettings = new JsonSerializerSettings
			{
				ContractResolver = new IgnorePropertiesResolver(arrayProps.Select(prop => prop.Name))
			};
			this.serializer = JsonSerializer.Create(serializerSettings);
		}
	}
}
