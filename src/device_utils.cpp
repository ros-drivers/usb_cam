#include <usb_cam/device_utils.h>
#include <usb_cam/usb_cam.h>

#include <dirent.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <linux/media.h>

#include <string.h>
#include <vector>
#include <sstream>

using dev_vec = std::vector<std::string>;
using dev_map = std::map<std::string, std::string>;

static bool is_v4l_dev(const char *name)
{
  return !memcmp(name, "video", 5) ||
    !memcmp(name, "radio", 5) ||
    !memcmp(name, "vbi", 3) ||
    !memcmp(name, "v4l-subdev", 10);
}

static int calc_node_val(const char *s)
{
  int n = 0;

  s = strrchr(s, '/') + 1;
  if (!memcmp(s, "video", 5))
  {
    n = 0;
  }
  else if (!memcmp(s, "radio", 5))
  {
    n = 0x100;
  }
  else if (!memcmp(s, "vbi", 3))
  {
    n = 0x200;
  }
  else if (!memcmp(s, "v4l-subdev", 10))
  {
    n = 0x300;
  } 
  n += atol(s + (n >= 0x200 ? 3 : 5));
  return n;
}

static bool sort_on_device_name(const std::string &s1, const std::string &s2)
{
  int n1 = calc_node_val(s1.c_str());
  int n2 = calc_node_val(s2.c_str());
  return n1 < n2;
}

static std::string get_first_device_filename(const std::string& input)
{
  //  example of input
  /*  
Intel(R) RealSense(TM) 415: Int (usb-0000:00:14.0-1):
	/dev/video2
	/dev/video3
	/dev/video4
  */
  std::string result = "";
  ssize_t found = input.find("/dev");
  if (found > 0)
  {
    std::string tmp = input.substr(found);
    result = tmp.substr(0, tmp.find("\n"));
  }

  return result;
}

static void update_device_filename(std::map<std::string, std::string>& map_dev_serial)
{
  DIR *dp;
  struct dirent *ep;
  dev_vec files;
  dev_map links;
  dev_map cards;
  struct v4l2_capability vcap;

  dp = opendir("/dev");
  if (dp == nullptr)
  {
    //  Couldn't open the directory
    return;
  }

  while ((ep = readdir(dp)))
  {
    if (is_v4l_dev(ep->d_name))
    {
      files.push_back(std::string("/dev/") + ep->d_name);
    }
  }

  closedir(dp);

  /* Find device nodes which are links to other device nodes */
  for (auto iter = files.begin(); iter != files.end(); )
  {
    char link[64+1];
    int link_len;
    std::string target;

    link_len = readlink(iter->c_str(), link, 64);
    if (link_len < 0) /* Not a link or error */
    {
      iter++;
      continue;
    }
    link[link_len] = '\0';

    /* Only remove from files list if target itself is in list */
    if (link[0] != '/') /* Relative link */
    {
      target = std::string("/dev/");
    }
    target += link;

    if (find(files.begin(), files.end(), target) == files.end())
    {
      iter++;
      continue;
    }

    /* Move the device node from files to links */
    if (links[target].empty())
    {
      links[target] = *iter;
    }
    else
    {
      links[target] += ", " + *iter;
    }

    iter = files.erase(iter);
  }

  std::sort(files.begin(), files.end(), sort_on_device_name);

  for (const auto &file : files)
  {
    int fd = open(file.c_str(), O_RDWR);
    std::string bus_info;
    std::string card;

    if (fd < 0)
    {
      continue;
    }
    
    int err = ioctl(fd, VIDIOC_QUERYCAP, &vcap);
    if (err)
    {
      struct media_device_info mdi;

      err = ioctl(fd, MEDIA_IOC_DEVICE_INFO, &mdi);
      if (!err)
      {
        if (mdi.bus_info[0])
        {
          bus_info = mdi.bus_info;
        }
        else
        {
          bus_info = std::string("platform:") + mdi.driver;
        }
        
        if (mdi.model[0])
        {
          card = mdi.model;
        }
        else
        {
          card = mdi.driver;
        }
      }
    } else
    {
      bus_info = reinterpret_cast<const char *>(vcap.bus_info);
      card = reinterpret_cast<const char *>(vcap.card);
    }

    close(fd);
    if (err)
    {
      continue;
    }

    if (cards[bus_info].empty())
    {
      cards[bus_info] += card + " (" + bus_info + "):\n";
    }
    cards[bus_info] += "\t" + file;

    if (!(links[file].empty()))
    {
      cards[bus_info] += " <- " + links[file];
    }
    cards[bus_info] += "\n";
  }

  for (const auto &card : cards)
  {
    std::string dev_name = get_first_device_filename(card.second);
    map_dev_serial.insert(std::pair<std::string, std::string>(dev_name,""));
  }
}

static void update_serial_number(std::map<std::string, std::string>& map_dev_serial)
{
  std::map<std::string, std::string>::iterator it = map_dev_serial.begin();
  for (; it != map_dev_serial.end(); ++it)
  {
    // build the command
    std::stringstream ss;
    ss << "udevadm info -a -p $(udevadm info -q path -n " << it->first << ")";
    std::string cmd = ss.str();

    // capture the output
    std::string output;
    int buffer_size = 256;
    char buffer[buffer_size];
    FILE *stream = popen(cmd.c_str(), "r");
    if (stream)
    {
      while (!feof(stream))
      {
        if (fgets(buffer, buffer_size, stream) != NULL)
        {
          output.append(buffer);
        }
      }
      pclose(stream);

      std::size_t found = output.find("ATTRS{serial}");
      if (found > 0)
      {
        //  get the full line with serial number information
        std::string tmp = output.substr(found);
        std::string sn_line = tmp.substr(0, tmp.find("\n"));

        //  removing double quotes
        ssize_t start = sn_line.rfind("=\"") + 2;
        std::string sn_clean = sn_line.substr(start, sn_line.rfind("\"")-start);
        it->second = sn_clean;

        //  only the first one, because udevadm looks for all parents devices
        continue;
      }
    }
    else
    {
      //  usb_cam_node could not run the command
    }
  }
}

void get_dev_serial_info(std::map<std::string, std::string>& map_dev_serial)
{
  //  it has to get the device file name first
  update_device_filename(map_dev_serial);

  //  with the updated device file name it is possible to get the serial number
  update_serial_number(map_dev_serial);
}
