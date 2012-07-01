using System;

namespace ArdupilotMega.Presenter
{
    /// <summary>
    /// A command that executes delegates to determine whether the command can execute, and to execute the command.
    /// </summary>
    /// <remarks>
    /// <para>
    /// This command implementation is useful when the command simply needs to execute a method on a view model. The delegate for
    /// determining whether the command can execute is optional. If it is not provided, the command is considered always eligible
    /// to execute.
    /// </para>
    /// </remarks>
    public class DelegateCommand : ICommand
    {
        private readonly Predicate<object> _canExecute;
        private readonly Action<object> _execute;

        public DelegateCommand(Action<object> execute)
        {
            _execute = execute;
        }

        /// <summary>
        /// Constructs an instance of <c>DelegateCommand</c>.
        /// </summary>
        /// <param name="execute">
        /// The delegate to invoke when the command is executed.
        /// </param>
        /// <param name="canExecute">
        /// The delegate to invoke to determine whether the command can execute.
        /// </param>
        public DelegateCommand(Action<object> execute, Predicate<object> canExecute)
        {
            _execute = execute;
            _canExecute = canExecute;
        }

        /// <summary>
        /// Determines whether this command can execute.
        /// </summary>
        public bool CanExecute(object parameter)
        {
            if (_canExecute == null)
            {
                return true;
            }

            return _canExecute(parameter);
        }

        /// <summary>
        /// Executes this command.
        /// </summary>
        public void Execute(object parameter)
        {
            _execute(parameter);
        }
    }
}