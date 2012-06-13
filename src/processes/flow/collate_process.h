/*ckwg +5
 * Copyright 2011-2012 by Kitware, Inc. All Rights Reserved. Please refer to
 * KITWARE_LICENSE.TXT for licensing information, or contact General Counsel,
 * Kitware, Inc., 28 Corporate Drive, Clifton Park, NY 12065.
 */

#ifndef VISTK_PROCESSES_FLOW_COLLATE_PROCESS_H
#define VISTK_PROCESSES_FLOW_COLLATE_PROCESS_H

#include "flow-config.h"

#include <vistk/pipeline/process.h>

#include <boost/scoped_ptr.hpp>

/**
 * \file collate_process.h
 *
 * \brief Declaration of the collate process.
 */

namespace vistk
{

/**
 * \class collate_process
 *
 * \brief A process for collating input data from multiple input edges.
 *
 * \note Edges for a \portvar{tag} may \em only be connected after the
 * \port{color/\portvar{tag}} is connected to. Before this connection happens,
 * the other ports to not exist and will cause errors. In short: The first
 * connection for any \portvar{tag} must be \port{color/\portvar{tag}}.
 *
 * \process Collate incoming data into a single stream.
 *
 * \iports
 *
 * \iport{color/\portvar{tag}} The color of the result \portvar{tag}.
 * \iport{coll/\portvar{tag}/\portvar{group}} A port to collate data for
 *                                            \portvar{tag} from. Data is
 *                                            collated from ports in
 *                                            ASCII-betical order.
 *
 * \oports
 *
 * \oport{res/\portvar{tag}} The collated result \portvar{tag}.
 *
 * \reqs
 *
 * \req Each input port \port{color/\portvar{tag}} must be connected.
 * \req Each \portvar{tag} must have at least two inputs to collate.
 * \req Each output port \port{res/\portvar{tag}} must be connected.
 *
 * \todo Add configuration to allow forcing a number of inputs for a result.
 * \todo Add configuration to allow same number of sources for all results.
 *
 * \ingroup process_flow
 */
class VISTK_PROCESSES_FLOW_NO_EXPORT collate_process
  : public process
{
  public:
    /**
     * \brief Constructor.
     *
     * \param config The configuration for the process.
     */
    collate_process(config_t const& config);
    /**
     * \brief Destructor.
     */
    ~collate_process();
  protected:
    /**
     * \brief Initialize the process.
     */
    void _init();

    /**
     * \brief Reset the process.
     */
    void _reset();

    /**
     * \brief Step the process.
     */
    void _step();

    /**
     * \brief The constraints on the process.
     */
    constraints_t _constraints() const;

    /**
     * \brief Input port information.
     *
     * \param port The port to get information about.
     *
     * \returns Information about an input port.
     */
    port_info_t _input_port_info(port_t const& port);
  private:
    class priv;
    boost::scoped_ptr<priv> d;
};

}

#endif // VISTK_PROCESSES_FLOW_COLLATE_PROCESS_H
