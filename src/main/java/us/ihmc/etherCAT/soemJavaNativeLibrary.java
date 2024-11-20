package us.ihmc.etherCAT;

import us.ihmc.tools.nativelibraries.NativeLibraryDescription;
import us.ihmc.tools.nativelibraries.NativeLibraryLoader;
import us.ihmc.tools.nativelibraries.NativeLibraryWithDependencies;

public class soemJavaNativeLibrary implements NativeLibraryDescription
{
   @Override
   public String getPackage(OperatingSystem os, Architecture arch)
   {
      return "us.ihmc.soem.generated";
   }

   @Override
   public NativeLibraryWithDependencies getLibraryWithDependencies(OperatingSystem os, Architecture arch)
   {
      if (os == OperatingSystem.LINUX64 && arch == Architecture.x64)
      {
         return NativeLibraryWithDependencies.fromFilename("libsoemJava-x86_64.so");
      }
      else if (os == OperatingSystem.LINUX64 && arch == Architecture.arm64)
      {
         return NativeLibraryWithDependencies.fromFilename("libsoemJava-arm64.so");
      }
      throw new RuntimeException("Unsupported platform: " + os.name() + "-" + arch.name());
   }

   private static boolean loaded = false;

   public static boolean load()
   {
      if (!loaded)
      {
         soemJavaNativeLibrary lib = new soemJavaNativeLibrary();
         loaded = NativeLibraryLoader.loadLibrary(lib);
      }
      return loaded;
   }
}