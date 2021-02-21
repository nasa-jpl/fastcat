#ifndef FASTCAT_ATI_FTS_OFFLINE_H_
#define FASTCAT_ATI_FTS_OFFLINE_H_

// Include related header (for cc files)

// Include c then c++ libraries

// Include external then project includes
#include "fastcat/jsd/ati_fts.h"

namespace fastcat
{
class AtiFtsOffline: public AtiFts
{
 public:
  bool ConfigFromYaml(YAML::Node node) override;
  bool Read() override;
};

}  // namespace fastcat

#endif
