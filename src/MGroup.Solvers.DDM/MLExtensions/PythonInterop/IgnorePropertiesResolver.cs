namespace MGroup.Solvers.DDM.MLExtensions.PythonInterop
{
	using System;
	using System.Collections.Generic;
	using System.Reflection;
	using System.Text;

	using Newtonsoft.Json;
	using Newtonsoft.Json.Serialization;

	public class IgnorePropertiesResolver : DefaultContractResolver
    {
		private readonly HashSet<string> propsToIgnore;
		public IgnorePropertiesResolver(IEnumerable<string> propertyNamesToIgnore)
		{
			this.propsToIgnore = new HashSet<string>(propertyNamesToIgnore);
		}

		protected override JsonProperty CreateProperty(MemberInfo member, MemberSerialization memberSerialization)
		{
			JsonProperty property = base.CreateProperty(member, memberSerialization);
			if (propsToIgnore.Contains(property.PropertyName))
			{
				property.ShouldSerialize = _ => false;
			}
			return property;
		}
	}
}
