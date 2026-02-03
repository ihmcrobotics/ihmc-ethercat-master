# Making a Release of IHMC EtherCAT Master
1. Bump the version in `build.gradle.kts`
2. Commit only `build.gradle.kts` with a message in the format: "`:bookmark: <version>`" Example: `git commit -m ":bookmark: <version>"`
3. Create a tag with the version name `git tag <version>`
4. Push the release commit and tag
    1. `git push`
    2. `git push origin <version>`
5. Setup Publishing according to [Confluence](https://ihmcrobotics.atlassian.net/wiki/spaces/PUBLIC/pages/14844236/Publishing+Maven+Artifacts).
6. Publish using `gradle compositePublish -PpublishUrl=robotlabfiles`
7. Create a release on GitHub documenting the changes (following the format of existing releases)
8. Announce the release to whoever may be interested
