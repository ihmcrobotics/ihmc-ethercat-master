plugins {
   id("us.ihmc.ihmc-build")
   id("us.ihmc.log-tools-plugin") version "0.6.4"
}

ihmc {
   group = "us.ihmc"
   version = "0.15.0"
   vcsUrl = "https://github.com/ihmcrobotics/ihmc-ethercat-master"
   openSource = true

   configureDependencyResolution()
   resourceDirectory("main", "swig")
   configurePublications()
}

app.entrypoint("SlaveInfo", "us.ihmc.etherCAT.master.SlaveInfo")

mainDependencies {
   api("us.ihmc:soem:1.5.0")
   api("us.ihmc:soem-platform-linux-x86_64:1.5.0")
   api("us.ihmc:soem-platform-linux-arm64:1.5.0")
   api("us.ihmc:ihmc-native-library-loader:2.0.3")
   api("us.ihmc:ihmc-realtime:1.6.0")
}

