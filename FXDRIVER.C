#include "FXDRIVER.H"

extern DPT driver$dpt;
extern DDT driver$ddt;
extern FDT driver$fdt;

// PCI bus ADP is FFFFFFFF.81D1F7C0 

int driver$init_tables()
{
    // Populate the DPT.
    ini_dpt_name        (&driver$dpt, "SSTDRIVER");
    ini_dpt_ucbsize     (&driver$dpt, sizeof(SST_UCB));
    ini_dpt_defunits    (&driver$dpt, 1 );
    ini_dpt_adapt       (&driver$dpt, AT$_PCI);
    ini_dpt_struc_init  (&driver$dpt, SST$struc_init);      // called to initialize the driver's structures on load
    ini_dpt_struc_reinit(&driver$dpt, SST$struc_reinit);    // called to reinitialize the driver's structures
    ini_dpt_end         (&driver$dpt);

    // Populate the DDT. These functions get called by the kernel.
    ini_ddt_unitinit    (&driver$ddt, SST$unit_init);
    ini_ddt_start       (&driver$ddt, SST$startio);
    ini_ddt_cancel      (&driver$ddt, ioc_std$cancelio);
    ini_ddt_end         (&driver$ddt);

    /*
    ini_fdt_act (&driver$fdt, IO$_WRITELBLK, lr$write, BUFFERED_64);
    ini_fdt_act (&driver$fdt, IO$_WRITEPBLK, lr$write, BUFFERED_64);
    ini_fdt_act (&driver$fdt, IO$_WRITEVBLK, lr$write, BUFFERED_64);
    ini_fdt_act (&driver$fdt, IO$_SETMODE, lr$setmode, BUFFERED_64);
    ini_fdt_act (&driver$fdt, IO$_SETCHAR, lr$setmode, BUFFERED_64);
    ini_fdt_act (&driver$fdt, IO$_SENSEMODE, exe_std$sensemode, BUFFERED_64);
    ini_fdt_act (&driver$fdt, IO$_SENSECHAR, exe_std$sensemode, BUFFERED_64);
    ini_fdt_end (&driver$fdt);
    */

    return SS$_NORMAL;
}

/* Skeleton driver functions. */

// unit_init - Unit initialization entry point
// Called to initialize an individual device unit.
// This should fork() and do its work in another process.
int SST$unit_init(  IDB *idb,			    /* Interrupt Data Block pointer			*/
		            SST_UCB * ucb )		    /* Unit Control Block pointer			*/
{
    char    tmpstr[255];

    ADP     *adp = ucb->ucb$r_ucb.ucb$ps_adp;
    CRB     *crb = ucb->ucb$r_ucb.ucb$l_crb;

    int vendorid, deviceid;
    int status;

    // We can't use C RTL functions from inside the kernel.
    SST$print_message(ucb, "unit_init: ADP is ", STS$K_INFO, FALSE);
    exe_std$outhex((uint64)adp);
    exe_std$outcrlf();

    SST$print_message(ucb, "unit_init: CRB is ", STS$K_INFO, FALSE);
    exe_std$outhex((uint64)crb);
    SST$print_message(ucb, ", node is ", STS$K_INFO, FALSE);
    exe_std$outhex((uint64)crb->crb$l_node);
    exe_std$outcrlf();

    // confirm this is the right device
    status = ioc$read_pci_config(adp,
                        crb->crb$l_node,
                        0,
                        2,
                        (int *)&vendorid);

    status = ioc$read_pci_config(adp,
                        crb->crb$l_node,
                        2,
                        2,
                        (int *)&deviceid);

    SST$print_message(ucb, "unit_init: vendor id is ", STS$K_INFO, FALSE);
    exe_std$outhex(vendorid);
    SST$print_message(ucb, "unit_init: device id is ", STS$K_INFO, FALSE);
    exe_std$outhex(deviceid);
    exe_std$outcrlf();

    return( SS$_NORMAL );
}

void SST$startio (IRP *irp, SST_UCB *ucb) {
    // skeleton 
    return;
}

// Called upon initial load. Set up any structures the driver needs.
void SST$struc_init (CRB *crb, DDB *ddb, IDB *idb, ORB *orb, SST_UCB *ucb) {
    ucb->ucb$r_ucb.ucb$b_devclass = DC$_VIDEO;
    SST$print_message(ucb, "struc_init called. makima is listening", STS$K_INFO, TRUE);

    return;
}

void SST$struc_reinit ( CRB *crb,		/* Channel request block			*/
                        DDB *ddb,		/* Device data block				*/
		                IDB *idb,		/* Interrupt dispatch block			*/
		                ORB *orb,		/* Object rights block				*/
		                UCB *ucb )		/* Unit control block				*/
{
    return;
}

void SST$print_message(SST_UCB *ucb, char *message, int severity, int crlf)
{
    DDB		*ddb;				/* Device Data Block pointer			*/

    ddb = ucb->ucb$r_ucb.ucb$l_ddb;

    /* Ignore informational messages unless the SYSGEN UserD1
     * parameter is set.
     */
    /*
    if((( sgn$gl_userd1 & 1 ) == 0 ) &&
       ( severity == STS$K_INFO )) {
	return;
    }
    */
    
    /* Print out a message prefix using the severity code
     * and device name from the DDB.
     */
    exe_std$outzstring( "\a\r\n%SSTDRIVER-" );
    switch( severity ) {
    case STS$K_INFO:
    default:
	exe_std$outzstring( "I- " );
	break;
    case STS$K_WARNING:
	exe_std$outzstring( "W- " );
	break;
    case STS$K_ERROR:
	exe_std$outzstring( "E- " );
	break;
    case STS$K_SEVERE:
	exe_std$outzstring( "F- " );
	break;
    }
    exe_std$outcstring( ddb->ddb$t_name );
    exe_std$outzstring( "0, " );

    /* Print the user-provided portion of the message followed
     * by CR-LF.
     */
    exe_std$outzstring( message );
    if(crlf) exe_std$outcrlf();
    return;
}