namespace ArdupilotMega.Presenter
{
    public interface ICommand
    {
        void Execute(object param);
        bool CanExecute(object param);
    }
}