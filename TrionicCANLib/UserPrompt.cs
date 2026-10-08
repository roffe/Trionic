using System;

namespace TrionicCANLib.API
{
    /// <summary>
    /// Questions the library needs a human to answer mid-operation (the library used to call
    /// WinForms MessageBox directly). The GUI installs these; they may be invoked from a worker
    /// thread and must block until the user answers.
    /// </summary>
    public static class UserPrompt
    {
        /// <summary>(text, caption) → true for Yes. Unset → No, so headless callers never block.</summary>
        public static Func<string, string, bool> YesNo;

        /// <summary>(text, caption) → shown to the user. Unset → no-op.</summary>
        public static Action<string, string> Notify;

        public static bool AskYesNo(string text, string caption) => YesNo?.Invoke(text, caption) ?? false;

        public static void Show(string text, string caption = "") => Notify?.Invoke(text, caption);
    }
}
