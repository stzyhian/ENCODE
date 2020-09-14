import os
import numpy as np
import torch
from torch.utils.data import Dataset
from laserscan import LaserScan
from scipy.spatial.transform import Rotation as R

EXTENSIONS_SCAN = ['.bin']


def is_scan(filename):
    return any(filename.endswith(ext) for ext in EXTENSIONS_SCAN)

class SlamKitti(Dataset):

    def __init__(self, root,    # directory where data is
                 sequences,     # sequences for this data (e.g. [1,3,4,6])
                 sensor,              # sensor to parse scans from
                 max_points=150000):   # max number of points present in dataset
        # save deats
        self.root = os.path.join(root, "sequences")
        self.sequences = sequences
        self.sensor = sensor
        self.sensor_img_H = sensor["img_prop"]["height"]
        self.sensor_img_W = sensor["img_prop"]["width"]
        self.sensor_img_means = torch.tensor(sensor["img_means"],
                                             dtype=torch.float)
        self.sensor_img_stds = torch.tensor(sensor["img_stds"],
                                            dtype=torch.float)
        self.sensor_fov_up = sensor["fov_up"]
        self.sensor_fov_down = sensor["fov_down"]
        self.max_points = max_points

        # make sure directory exists
        if os.path.isdir(self.root):
            print("Sequences folder exists! Using sequences from %s" % self.root)
        else:
            raise ValueError("Sequences folder doesn't exist! Exiting...")

        # make sure sequences is a list
        assert(isinstance(self.sequences, list))

        # placeholder for filenames
        self.scan_files = []

        # fill in with names, checking that all sequences are complete
        for seq in self.sequences:
            # to string
            seq = '{0:02d}'.format(int(seq))

            print("parsing seq {}".format(seq))

            # get paths for each
            scan_path = os.path.join(self.root, seq, "velodyne")

            # get files
            scan_files = [os.path.join(dp, f) for dp, dn, fn in os.walk(
                os.path.expanduser(scan_path)) for f in fn if is_scan(f)]

            # extend list
            self.scan_files.extend(scan_files)

        # sort for correspondance
        self.scan_files.sort()

        print("Using {} scans from sequences {}".format(len(self.scan_files),
                                                        self.sequences))

    def getPointCloud(self, index):
        scan_file = self.scan_files[index]
        scan = np.fromfile(scan_file, dtype=np.float32)
        scan = scan.reshape((-1, 4))
        return scan
    
    def getSingleItem(self, index):
        # get item in tensor shape
        scan_file = self.scan_files[index]

        # open a laserscan
        scan = LaserScan(project=True,
                         H=self.sensor_img_H,
                         W=self.sensor_img_W,
                         fov_up=self.sensor_fov_up,
                         fov_down=self.sensor_fov_down)

        # open and obtain scan
        scan.open_scan(scan_file)

        # get points and labels
        proj_range = torch.from_numpy(scan.proj_range).clone()
        proj_xyz = torch.from_numpy(scan.proj_xyz).clone()
        proj_remission = torch.from_numpy(scan.proj_remission).clone()
        proj_mask = torch.from_numpy(scan.proj_mask)

        proj = torch.cat([proj_range.unsqueeze(0).clone(),
                            proj_xyz.clone().permute(2, 0, 1),
                            proj_remission.unsqueeze(0).clone()])
        proj = (proj - self.sensor_img_means[:, None, None]
                ) / self.sensor_img_stds[:, None, None]
        proj = proj * proj_mask.float()

        return proj

    def __getitem__(self, index):
        next_index = index + 1
        scan0 = self.getSingleItem(index)
        scan1 = self.getSingleItem(next_index)

        return scan0, scan1

    def __len__(self):
        return len(self.scan_files) - 1


class Parser():
    # standard conv, BN, relu
    def __init__(self,
                 root,              # directory for data
                 test_sequences,    # sequences to test
                 sensor,            # sensor to use
                 max_points,        # max points in each scan in entire dataset
                 batch_size,        # batch size for train and val
                 workers):          # threads to load data
        super(Parser, self).__init__()

        # if I am training, get the dataset
        self.root = root
        self.test_sequences = test_sequences
        self.sensor = sensor
        self.max_points = max_points
        self.batch_size = batch_size
        self.workers = workers

        self.test_dataset = SlamKitti(root=self.root,
                                        sequences=self.test_sequences,
                                        sensor=self.sensor,
                                        max_points=max_points)

        self.testloader = torch.utils.data.DataLoader(self.test_dataset,
                                                        batch_size=self.batch_size,
                                                        shuffle=False,
                                                        num_workers=self.workers,
                                                        pin_memory=True,
                                                        drop_last=True)
        assert len(self.testloader) > 0
        self.testiter = iter(self.testloader)

    def get_test_batch(self):
        scans = self.testiter.next()
        return scans

    def get_test_set(self):
        return self.testloader

    def get_test_size(self):
        return len(self.testloader)
    
    def get_test_dataset(self):
        return self.test_dataset
