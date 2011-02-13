using System;

namespace ArducopterConfigurator
{
    public abstract class VmBase : NotifyProperyChangedBase
    {
        protected string[] PropsInUpdateOrder;

        // Common method for creating the update data
        // sentence sent to APM is the commandChar followed by the property
        // vals in the correct order, seperated by semicolons
        protected string ComposePropsWithCommand(string commandChar)
        {
            var strings = new string[PropsInUpdateOrder.Length];
            for (int i = 0; i < PropsInUpdateOrder.Length; i++)
            {
                var prop = this.GetType().GetProperty(PropsInUpdateOrder[i]);

                if (prop.PropertyType == typeof(bool))
                    strings[i] = ((bool)prop.GetValue(this, null)) ? "1" : "0";
                else
                    strings[i] = prop.GetValue(this, null).ToString();

            }

            return commandChar + string.Join(";", strings);
        }

        // Common method for populating properties, using a hardcoded 
        // property update order, and reflection to get the property type
        protected void PopulatePropsFromUpdate(string strRx, bool fireInpc)
        {

            var strs = strRx.Split(',');

            if (PropsInUpdateOrder.Length != strs.Length)
            {
                Console.WriteLine("Processing update with " + strs.Length
                                  + " values, but have " + PropsInUpdateOrder.Length
                                  + " properties to populate. Ignoring this update");
                return;
            }

            for (int i = 0; i < PropsInUpdateOrder.Length; i++)
            {
                var prop = this.GetType().GetProperty(PropsInUpdateOrder[i]);
                var s = strs[i];
                object value = null;

                if (prop == null)
                {
                    Console.WriteLine("Trying to set non existant property: " + PropsInUpdateOrder[i]);
                    break;
                }

                if (prop.PropertyType == typeof(float))
                {
                    float val;
                    if (!float.TryParse(s, out val))
                    {
                        Console.WriteLine("Error parsing float: {0}, VM: {1}" + s, "TODO");
                        break;
                    }
                    value = val;
                }
                if (prop.PropertyType == typeof(bool))
                {
                    float val;
                    if (!float.TryParse(s, out val))
                    {
                        Console.WriteLine("Error parsing float (bool): {0}, VM: {1}" + s, "TODO");
                        break;
                    }
                    value = val != 0.0;
                }

                if (prop.PropertyType == typeof(int))
                {
                    int val;
                    if (!int.TryParse(s, out val))
                    {
                        Console.WriteLine("Error parsing int:{0}, VM: {1}" + s, "TODO");
                        break;
                    }
                    value = val;
                }

                prop.SetValue(this, value, null);

                if (fireInpc)
                    FirePropertyChanged(PropsInUpdateOrder[i]);
            }
        }

    }
}