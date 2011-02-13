using System;

namespace ArducopterConfigurator
{
    public interface IPresentationModel : ItalksToApm
    {
        string Name { get; }
        void Activate();
        void DeActivate();
        
        event EventHandler updatedByApm;
    }
}