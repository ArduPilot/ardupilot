using System.Windows.Forms;

namespace ArducopterConfigurator
{
    public interface IView<Tmodel> where Tmodel : IPresentationModel
    {
        void SetDataContext(Tmodel vm);
        Control Control { get; }
    }
}