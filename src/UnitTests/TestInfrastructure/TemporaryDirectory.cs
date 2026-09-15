using System;
using System.IO;

namespace UnitTests.TestInfrastructure
{
    internal sealed class TemporaryDirectory : IDisposable
    {
        private bool _disposed;

        internal TemporaryDirectory()
        {
            Path = System.IO.Path.Combine(
                System.IO.Path.GetTempPath(),
                "NurbsSharp.UnitTests",
                Guid.NewGuid().ToString("N"));
            Directory.CreateDirectory(Path);
        }

        internal string Path { get; }

        public void Dispose()
        {
            if (_disposed)
                return;

            if (Directory.Exists(Path))
                Directory.Delete(Path, recursive: true);

            _disposed = true;
        }
    }
}
