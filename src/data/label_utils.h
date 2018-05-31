#ifndef LABEL_UTILS_H_
#define LABEL_UTILS_H_

/**
 * \brief Tools for processing of label description files.
 *
 * A label description file consists of names and colors for labels represented by a numerical id. The xml
 * structure is defined as follows:
 *
 * <config>
 *   <label>
 *     <id>0</id>
 *     <name>no label</name>
 *     <description>all points with no special label.</description>
 *     <color>255 255 255</color>
 *   </label>
 *   <label>
 *     [...]
 *   </label>
 * </config>
 */

#include <map>
#include <string>

#include <glow/GlColor.h>

/** \brief retrieve label names from given xml file.
 *  \brief param[in]    filename of the description file
 *  \brief param[out]   map of label ids to names.
 */
void getLabelNames(const std::string& filename, std::map<uint32_t, std::string>& label_names);

/** \brief retrieve label colors from given xml file.
 *  \brief param[in]    filename of the description file.
 *  \brief param[out]   map of labels to colors.
 */
void getLabelColors(const std::string& filename, std::map<uint32_t, glow::GlColor>& label_colors);

#endif /* LABEL_UTILS_H_ */
