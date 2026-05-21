from huggingface_hub import snapshot_download
import os, zipfile

data_dir = os.path.expanduser('~/ros2_ws/src/perception_pipeline/data/tless')

snapshot_download(
    repo_id='bop-benchmark/tless',
    allow_patterns=[
        'tless_base.zip',
        'tless_models.zip',
        'tless_test_primesense_bop19.zip'
    ],
    repo_type='dataset',
    local_dir=data_dir
)

# unzip all
for f in ['tless_base.zip', 'tless_models.zip', 'tless_test_primesense_bop19.zip']:
    zip_path = os.path.join(data_dir, f)
    print(f"Unzipping {f}...")
    with zipfile.ZipFile(zip_path, 'r') as z:
        z.extractall(data_dir)
    print(f"Done: {f}")

print("All files extracted.")