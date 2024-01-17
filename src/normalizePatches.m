function normalized_patches = normalizePatches(patches,hwid)
   d=(2*hwid+1)^2;
   mean_patches = ones(d,1) .* mean(patches,1);
   std_patches = ones(d,1) .* std(patches, 1);
   normalized_patches = (patches-mean_patches)./std_patches;
end