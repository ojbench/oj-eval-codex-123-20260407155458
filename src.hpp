// RAID5 implementation for ACMOJ driver-included main.cpp
#include <vector>
#include <fstream>
#include <cstring>

namespace sjtu { using fstream = std::fstream; }

// Disk event type
enum class EventType { NORMAL, FAILED, REPLACED };

class RAID5Controller {
private:
  std::vector<sjtu::fstream*> drives_;
  int blocks_per_drive_ = 0;
  int block_size_ = 4096;
  int num_disks_ = 0;
  // -1 means no failed or replaced drive currently
  int failed_drive_ = -1;

  // Map global data block id to stripe and disk positions
  inline void locate(int block_id, int &stripe, int &parity_disk, int &data_disk, int &offset_in_stripe) const {
    stripe = block_id / (num_disks_ - 1);
    offset_in_stripe = block_id % (num_disks_ - 1);
    // Rotating parity by stripe index
    parity_disk = stripe % num_disks_;
    // Data disks are remaining disks in cyclic order after parity
    int idx = 0;
    for (int k = 0, d = (parity_disk + 1) % num_disks_; k < num_disks_; ++k, d = (d + 1) % num_disks_) {
      if (d == parity_disk) continue;
      if (idx == offset_in_stripe) { data_disk = d; break; }
      ++idx;
    }
  }

  inline std::streampos byte_offset(int stripe) const {
    long long pos_block = static_cast<long long>(stripe);
    return static_cast<std::streampos>(pos_block * block_size_);
  }

  void read_block_from_disk(int disk, int stripe, char *buf) {
    auto &fs = *drives_[disk];
    fs.seekg(byte_offset(stripe), std::ios::beg);
    fs.read(buf, block_size_);
    if (fs.gcount() != block_size_) {
      std::memset(buf, 0, block_size_);
      fs.clear();
    }
  }

  void write_block_to_disk(int disk, int stripe, const char *buf) {
    auto &fs = *drives_[disk];
    fs.seekp(byte_offset(stripe), std::ios::beg);
    fs.write(buf, block_size_);
    fs.flush();
  }

  void compute_parity_for_stripe(int stripe, char *parity_out) {
    std::vector<char> tmp(block_size_);
    std::memset(parity_out, 0, block_size_);
    int parity_disk = stripe % num_disks_;
    for (int d = 0; d < num_disks_; ++d) {
      if (d == parity_disk) continue; // skip parity block itself
      if (d == failed_drive_) continue; // missing drive contributes zeros
      read_block_from_disk(d, stripe, tmp.data());
      for (int i = 0; i < block_size_; ++i) parity_out[i] ^= tmp[i];
    }
  }

public:
  RAID5Controller(std::vector<sjtu::fstream*> drives, int blocks_per_drive, int block_size = 4096) {
    drives_ = std::move(drives);
    blocks_per_drive_ = blocks_per_drive;
    block_size_ = block_size;
    num_disks_ = static_cast<int>(drives_.size());
  }

  void Start(EventType event_type_, int drive_id) {
    if (event_type_ == EventType::NORMAL) {
      failed_drive_ = -1;
      return;
    }
    if (event_type_ == EventType::FAILED) {
      failed_drive_ = (drive_id >= 0 && drive_id < num_disks_) ? drive_id : -1;
      return;
    }
    if (event_type_ == EventType::REPLACED) {
      if (!(drive_id >= 0 && drive_id < num_disks_)) return;
      // Rebuild the replaced drive from existing data/parity
      failed_drive_ = drive_id; // treat as missing during reconstruction
      std::vector<char> buf(block_size_);
      for (int stripe = 0; stripe < blocks_per_drive_; ++stripe) {
        int parity_disk = stripe % num_disks_;
        if (drive_id == parity_disk) {
          // Recompute parity as XOR of all data blocks
          std::memset(buf.data(), 0, block_size_);
          std::vector<char> tmp(block_size_);
          for (int d = 0; d < num_disks_; ++d) {
            if (d == parity_disk) continue;
            if (d == failed_drive_) continue; // should not happen here
            read_block_from_disk(d, stripe, tmp.data());
            for (int i = 0; i < block_size_; ++i) buf[i] ^= tmp[i];
          }
          write_block_to_disk(drive_id, stripe, buf.data());
        } else {
          // Rebuild data block by XOR of parity and remaining data blocks
          std::memset(buf.data(), 0, block_size_);
          std::vector<char> tmp(block_size_);
          for (int d = 0; d < num_disks_; ++d) {
            if (d == drive_id) continue; // skip missing
            read_block_from_disk(d, stripe, tmp.data());
            for (int i = 0; i < block_size_; ++i) buf[i] ^= tmp[i];
          }
          write_block_to_disk(drive_id, stripe, buf.data());
        }
      }
      failed_drive_ = -1; // back to normal
      return;
    }
  }

  void Shutdown() {
    for (auto *p : drives_) {
      if (p && reinterpret_cast<std::fstream*>(p)->is_open()) {
        reinterpret_cast<std::fstream*>(p)->flush();
        reinterpret_cast<std::fstream*>(p)->close();
      }
    }
  }

  void ReadBlock(int block_id, char *result) {
    int stripe, parity_disk, data_disk, off;
    locate(block_id, stripe, parity_disk, data_disk, off);
    if (failed_drive_ < 0) {
      read_block_from_disk(data_disk, stripe, result);
      return;
    }
    if (failed_drive_ == data_disk) {
      // reconstruct data from parity and remaining data blocks
      std::memset(result, 0, block_size_);
      std::vector<char> tmp(block_size_);
      for (int d = 0; d < num_disks_; ++d) {
        if (d == data_disk) continue;
        if (d == failed_drive_) continue;
        read_block_from_disk(d, stripe, tmp.data());
        for (int i = 0; i < block_size_; ++i) result[i] ^= tmp[i];
      }
      return;
    }
    // failed drive is not data; read directly
    read_block_from_disk(data_disk, stripe, result);
  }

  void WriteBlock(int block_id, const char *data) {
    int stripe, parity_disk, data_disk, off;
    locate(block_id, stripe, parity_disk, data_disk, off);

    std::vector<char> newdata(block_size_);
    std::memcpy(newdata.data(), data, block_size_);

    if (failed_drive_ < 0) {
      // Normal RMW: new_parity = old_parity ^ old_data ^ new_data
      std::vector<char> olddata(block_size_), parity(block_size_);
      read_block_from_disk(data_disk, stripe, olddata.data());
      read_block_from_disk(parity_disk, stripe, parity.data());
      for (int i = 0; i < block_size_; ++i) parity[i] ^= (olddata[i] ^ newdata[i]);
      write_block_to_disk(data_disk, stripe, newdata.data());
      write_block_to_disk(parity_disk, stripe, parity.data());
      return;
    }

    // Degraded: one drive missing
    if (failed_drive_ == data_disk) {
      // Data disk missing: update parity to reflect new data; cannot write data block
      std::vector<char> parity(block_size_);
      compute_parity_for_stripe(stripe, parity.data());
      for (int i = 0; i < block_size_; ++i) parity[i] ^= newdata[i];
      write_block_to_disk(parity_disk, stripe, parity.data());
      return;
    } else if (failed_drive_ == parity_disk) {
      // Parity disk missing: write data; parity can be recomputed later
      write_block_to_disk(data_disk, stripe, newdata.data());
      return;
    } else {
      // Other disk missing: normal RMW for affected data/parity
      std::vector<char> olddata(block_size_), parity(block_size_);
      read_block_from_disk(data_disk, stripe, olddata.data());
      read_block_from_disk(parity_disk, stripe, parity.data());
      for (int i = 0; i < block_size_; ++i) parity[i] ^= (olddata[i] ^ newdata[i]);
      write_block_to_disk(data_disk, stripe, newdata.data());
      write_block_to_disk(parity_disk, stripe, parity.data());
      return;
    }
  }

  int Capacity() { return (num_disks_ - 1) * blocks_per_drive_; }
};

