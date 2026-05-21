from huggingface_hub import snapshot_download
import os

snapshot_download(
    repo_id='bop-benchmark/tless',
    allow_patterns=[
        'tless_base.zip',
        'tless_models.zip',
        'tless_test_primesense_bop19.zip'
    ],
    repo_type='dataset',
    local_dir=os.path.expanduser('~/ros2_ws/src/perception_pipeline/data/tless')
)