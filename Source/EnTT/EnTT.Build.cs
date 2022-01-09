using System.IO;
using UnrealBuildTool;

public class EnTT: ModuleRules {
    public EnTT(ReadOnlyTargetRules Target) : base(Target) {
        Type = ModuleType.External;
        PublicIncludePaths.Add(Path.Combine(ModuleDirectory, "entt"));
    }
}